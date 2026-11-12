# External validation

KAUHA is checked against numbers it did not produce itself: requirements from
standards, measurements published in papers, and recorded real-world data
replayed through the stack.

- Criteria: `kauha/validation/criteria.py`. Each has a source, a metric and a
  target, and was committed before its check was run.
- Checks: `kauha/validation/checks.py`.
- Results: `python -m kauha.validation.report` writes `results/validation.json`
  and [docs/results/validation.md](../results/validation.md). Every criterion is
  listed as pass, fail, finding, open or needs data. Failures stay in the table.

## Data that is not in the repository

Dataset checks read the directory named by an environment variable and are
reported as "needs data" when it is not set. Nothing is downloaded automatically.

| Variable | Dataset | Get it from | Licence |
|---|---|---|---|
| `KAUHA_DATA_ROAD` | ROAD CAN dataset (candump logs) | link in the paper, arXiv:2012.14600 | see the dataset page |
| `KAUHA_DATA_LIDARDUSTX` | LiDARDustX | https://huggingface.co/datasets/Weichenfeng/LiDARDustX | research use |
| `KAUHA_DATA_SODA` | SODA, YOLO layout (`images/`, `labels/`) with the person class id in `KAUHA_SODA_PERSON_CLASS` | link in arXiv:2202.09554 | CC BY 4.0 |
| `KAUHA_DATA_TUNI_LOADER` | Tampere University wheel loader recording | https://zenodo.org/records/10639199 | CC BY 4.0 |

What these checks cannot show: recorded data does not react to the machine, so
closed-loop behaviour still rests on the twin, the 1:8 testbed and field tests.
