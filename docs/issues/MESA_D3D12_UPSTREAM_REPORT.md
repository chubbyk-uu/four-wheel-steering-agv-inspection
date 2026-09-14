# Draft: d3d12 command-signature cache searches with a pointer-to-pointer, leaking signatures on repeated indirect draws

Status: prepared locally; not submitted upstream.

## Environment and scope

- Ubuntu 24.04 under WSL2, Mesa package `25.2.8-0ubuntu0.24.04.2`.
- Mesa D3D12 renderer, NVIDIA GeForce RTX 5080, host driver 616.92.
- Tested Gallium ELF Build ID: `6ccf17f6a28dda1c4c6e147c8d7503b974a501ba`.
- Reproduced without ROS, Gazebo, CUDA/OptiX, external assets, or image output.
- Other driver versions and native Windows have not been tested.

## Suspected source defect, confirmed by single-site experiment

In [`d3d12_get_cmd_signature()`](https://gitlab.freedesktop.org/mesa/mesa/-/blob/mesa-25.2.8/src/gallium/drivers/d3d12/d3d12_cmd_signature.cpp#L68), the hash-table search receives `&key` instead of `key`.
The hash and equality callbacks inspect the contents of `d3d12_cmd_signature_key`, but `key` is already a pointer to that structure.
Insertion correctly uses the copied structure inside `data`.

Consequently repeated requests for the same signature miss the lookup and call `CreateCommandSignature` again. [Hash-table insertion](https://gitlab.freedesktop.org/mesa/mesa/-/blob/mesa-25.2.8/src/util/hash_table.c#L494) with the same actual key replaces the entry's key/data pointers without disposing of the old wrapper and its COM reference. Cache destruction can only release the currently reachable entries.

Proposed source fix: [one-line patch](../../tools/patches/mesa-d3d12-command-signature-key.patch).

## Minimal reproducer

Source: [probe_mesa_indirect_memory.cpp](../../tools/probe_mesa_indirect_memory.cpp).

```bash
g++ -O2 -Wall -Wextra tools/probe_mesa_indirect_memory.cpp -o /tmp/mesa_probe -lGL -lX11
GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA /tmp/mesa_probe indirect 12000
GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA /tmp/mesa_probe direct 12000
```

Requires an X display and OpenGL 4.3. The program creates one 32×32 pbuffer, one shader program, one VAO and one fixed indirect argument buffer, then repeats the same triangle. Each iteration calls `glFinish()` and checks GL errors. It checks a foreground and background pixel at the end. The direct mode draws the same triangle with `glDrawArrays`.

| Mode | RSS at draw 2001 (KiB) | RSS at draw 10001 (KiB) | Final rendering check |
|---|---:|---:|---|
| Original, indirect | 173048 | 196648 | true |
| Original, direct | 164924 | 164924 | true |
| Single-site corrected library, indirect | 187768 | 187768 | true |

The original indirect mode grows by about 2.95 KiB/draw. The corrected mode stays flat. Absolute startup memory differs; the comparison concerns growth after initialization. The first final pixel readback introduces a one-time allocation, so these measurements use equal pre-readback draw counts.

## Allocation evidence

Heaptrack attached after initialization to the original application, with matching Ubuntu debug symbols, resolves the main retained allocation path to:

```text
glFenceSync → _mesa_fence_sync → tc_flush → _tc_sync → tc_batch_execute
  → d3d12_draw_vbo (d3d12_draw.cpp:1249)
  → d3d12_get_cmd_signature / inlined create_cmd_signature
  → CreateCommandSignature (d3d12_cmd_signature.cpp:59)
  → libd3d12core.so / libd3d12.so / NVIDIA WSL graphics user-mode driver
```

The fence call flushes queued draws; standalone correctly paired fence tests stay flat. Instrumenting the application's GL fence calls also shows a bounded live count (2000 created, 1995 deleted), rather than steadily accumulating GL sync objects.

## Correction experiment and limits

For causal diagnosis only, an exact local Gallium copy was changed at one instruction: the search argument loads the key pointer rather than taking the address of its stack slot. This is the assembly equivalent of the proposed source fix. The original file was not modified. The original unmodified library still grows when preloaded, controlling for preloading itself. This experiment is not a replacement for building and testing the source patch upstream.

In a separate 100 m Gazebo scene with GPU lidars and OptiX enabled, the original library grew at about 2.380 MiB/s during a stationary post-load window. The corrected copy stayed flat (4746.51 to 4746.25 MiB, fitted slope -0.00571 MiB/s). The final repeats used isolated ROS/Gazebo communication domains and no profiler. Both lidars continued publishing: 399/398 frames over 40 seconds with the original library and 398/397 with the corrected copy, each frame containing 1000×8 points. The corrected process was checked to map only the private Gallium copy. This was a short headless diagnostic, not a complete long-duration acceptance test.

Machine-independent results and exact binary provenance are in [the experiment record](../../results/mesa_d3d12_signature_root_cause.json). The source patch and reproducer can be attached directly to an upstream issue; local filesystem paths and raw profiling dumps are not needed.
