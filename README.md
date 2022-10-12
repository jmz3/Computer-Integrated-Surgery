# Programming Assignments for CIS

This is the repository for EN601.663 Computer Integrated Surgery @ Johns Hopkins University

This repository is created and maintained by
- Jiaming "Jeremy" Zhang: jzhan282@jhu.edu
- CHongjun Yang:          cyang94@jhu.edu

The general structure is borrowed from Benjamin's Template:

```bibtex
@misc{benjamindkilleen2022Sep,
 author = {Killeen, Benjamin D.},
 title = {{cispa: Template for CIS I programming assignments at Johns Hopkins}},
 journal = {GitHub},
 year = {2022},
 month = {Sep},
 url = {https://github.com/benjamindkilleen/cispa}
}
```


## Dependencies

This repo is developed via PYTHON 3.9.13 with the Dependencies as follows:

```PYTHON
black
flake8
pytest
numpy
click
rich
```

## Install

In a terminal, clone this repo:

```sh

```

Then change into the directory, create the Anacoconda environment, and activate it.

```bash
cd cispa
conda env create -f environment.yml
conda activate cispa
```

## Usage

Tell your users how to use your code. This should include how to run the code and how to run the tests.

```bash
python pa1.py -n pa1-debug-a
```

### Unit Tests

Use [pytest](https://docs.pytest.org/en/6.2.x/). In the `tests/` directory, place `.py` files that
start with `test_`, and contain functions that start with `test_`. Then use `assert` statements to
evaluate parts of your code.

Run all tests with

```sh
pytest -s
```

Or focus on a particular test, for example a function called `test_registration()` in `test_frame.py`:

```sh
pytest -s tests/test_frame.py::test_registration
```

The `-s` option tells pytest to allow `print` statements and other logging (use
[logging](https://docs.python.org/3/library/logging.html)!) to be passed through.
