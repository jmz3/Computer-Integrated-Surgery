# Programming Assignments for CIS

This is the repository for EN601.663 Computer Integrated Surgery @ Johns Hopkins University

This repository is created and maintained by
- Jiaming "Jeremy" Zhang: jzhan282@jhu.edu

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

In a terminal, clone this repo to a specified directory:

```sh
cd ~/YOUR_DIR
git clone https://github.com/jeremyzz830/Computer-Integrated-Surgery.git
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

TBD
