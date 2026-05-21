# Gemini 335L Camera

Use `orbbec_gemini335l` for Orbbec Gemini 335L cameras.

## Environment

Activate the existing conda environment before running any code:

```bash
source /home/zws/miniforge3/etc/profile.d/conda.sh
conda activate lerobot
```

## Quick check

```bash
python -m pytest tests/cameras/test_gemini335l.py -q
python tests/cameras/test_gemini335l.py
```

## Config type

Use `type: orbbec_gemini335l` in camera configuration files.
