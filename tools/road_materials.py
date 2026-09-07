"""Selected CC0 color sources and explicit project mapping scales."""
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MATERIAL = 'Concrete047A'
MATERIALS = {
    'gravel_concrete_03': dict(name='Gravel Concrete 03', file='gravel_concrete_03_diff_8k.png',
        url='https://polyhaven.com/a/gravel_concrete_03', width_m=2.1,
        scale_basis='provider physical width'),
    'Concrete047A': dict(name='Concrete 047 A', file='Concrete047A_8K-PNG_Color.png',
        url='https://ambientcg.com/view?id=Concrete047A', width_m=2.1,
        scale_basis='project comparison mapping; provider API dimensions are zero/unspecified'),
}
