"""Reuse expensive derived work across launches, keyed only by verified content.

Opening the 100 m scene spent about 14 s before a single simulator process
started: re-deriving collision-proxy geometry from 7.7 M triangles and
re-encoding 48 display textures, both pure functions of asset bytes that do
not change between runs.

The key is built from content the caller has already verified by SHA-256
against the manifest, plus a fingerprint of the checking code itself, so a hit
means the same bytes will meet the same checks. Nothing is keyed on a path, a
size or an mtime: a cache that can go stale when a file is edited in place is
not worth the seconds it saves here, and a cache hit must never skip an
integrity check - only the derivation behind it.

Set AGV_DERIVED_CACHE=off to force every launch to redo the work, and
AGV_DERIVED_CACHE_DIR to place the store somewhere other than the user cache.
"""
import hashlib
import json
import os
import shutil
import tempfile
from pathlib import Path

# Bumped by hand only for a store layout change; the checking code is
# fingerprinted into every key, so a logic change invalidates itself.
LAYOUT = 'v2'
_OFF = ('0', 'off', 'no', 'false')
_fingerprints = {}


def enabled():
    return os.environ.get('AGV_DERIVED_CACHE', '').strip().lower() not in _OFF


def root():
    override = os.environ.get('AGV_DERIVED_CACHE_DIR')
    if override:
        return Path(override)
    base = os.environ.get('XDG_CACHE_HOME') or Path.home()/'.cache'
    return Path(base)/'agv-4wids'


def code_fingerprint(*modules):
    """Hash the source of the modules whose checks the cached value stands for.

    Editing any of them changes every key derived from it, so a stale entry
    cannot outlive the logic that produced it.
    """
    names = tuple(sorted(m.__name__ for m in modules))
    if names not in _fingerprints:
        h = hashlib.sha256()
        for module in sorted(modules, key=lambda m: m.__name__):
            source = getattr(module, '__file__', None)
            if not source:
                raise ValueError('cannot fingerprint a module without source: '+module.__name__)
            h.update(module.__name__.encode())
            h.update(Path(source).read_bytes())
        _fingerprints[names] = h.hexdigest()
    return _fingerprints[names]


def key(*parts):
    """A stable digest of JSON-representable key material."""
    text = json.dumps(parts, sort_keys=True, separators=(',', ':'), default=str)
    return hashlib.sha256(text.encode()).hexdigest()


def _slot(namespace, digest):
    return root()/LAYOUT/namespace/digest


def read_value(namespace, digest):
    """The stored JSON value, or None. A damaged entry reads as a miss."""
    if not enabled():
        return None
    try:
        return json.loads((_slot(namespace, digest)/'value.json').read_text())
    except (OSError, ValueError):
        return None


def write_value(namespace, digest, value):
    """Publish atomically; a failed write is a miss next time, never a fault."""
    if not enabled():
        return
    slot = _slot(namespace, digest)
    try:
        slot.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=slot.parent) as staging:
            # Stage inside the temporary directory, never as it: moving the
            # directory itself away leaves its cleanup with nothing to remove.
            staged = Path(staging)/'entry'
            staged.mkdir()
            (staged/'value.json').write_text(json.dumps(value, indent=2))
            _replace(staged, slot)
    except OSError:
        pass


def read_directory(namespace, digest, destination):
    """Copy a stored directory into destination. False when there is no entry.

    The copy keeps the session self-contained, so an archived run does not
    depend on a cache that may be cleared later.
    """
    destination = Path(destination)
    if destination.exists() or destination.is_symlink():
        raise FileExistsError(str(destination))
    slot = _slot(namespace, digest)
    if not enabled() or not (slot/'.complete').is_file():
        return False
    # Claim a new destination ourselves. Never clean up a caller-owned directory.
    destination.mkdir()
    try:
        shutil.copytree(slot, destination, dirs_exist_ok=True)
        expected = json.loads((destination/'.complete').read_text())
        actual = _directory_hashes(destination)
        if not isinstance(expected, dict) or not expected or actual != expected:
            raise ValueError('damaged derived directory')
        (destination/'.complete').unlink()
        return True
    except (OSError, ValueError):
        shutil.rmtree(destination, ignore_errors=True)
        # Discard this damaged cache, so the next write can repair it.
        shutil.rmtree(slot, ignore_errors=True)
        return False


def _directory_hashes(directory):
    return {str(p.relative_to(directory)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(Path(directory).rglob('*'))
            if p.is_file() and p.name != '.complete'}


def write_directory(namespace, digest, source):
    """Store a finished directory. The marker is written last, so a partial
    copy left by a crash or a full disk is never read back as a hit."""
    if not enabled():
        return
    slot = _slot(namespace, digest)
    try:
        slot.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=slot.parent) as staging:
            staged = Path(staging)/'entry'
            shutil.copytree(source, staged)
            (staged/'.complete').write_text(json.dumps(_directory_hashes(staged)))
            _replace(staged, slot)
    except OSError:
        pass


def _replace(staged, slot):
    if slot.exists():                       # another process won the race
        return
    try:
        staged.replace(slot)
    except OSError:
        if not slot.exists():
            raise
