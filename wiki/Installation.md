# Installation

```bash
git clone https://github.com/timeout187/RigidFlightLab.git
cd RigidFlightLab
pip install -r requirements.txt
```

Requires Python 3.10+.

## Running the GUI

```bash
streamlit run src/gui/app.py
```

## Running the examples

```bash
python -m examples.nominal_run
python -m examples.dispersion_example
```

## Running the tests

```bash
python -m pytest tests/ -q
```
