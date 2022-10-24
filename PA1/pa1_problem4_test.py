import sys,os
sys.path.append(os.path.dirname(sys.path[0]))
from pathlib import Path
from cispa.Registration import regist_matched_points
from cispa.PivotCalibration import calib_pivot_points
import cispa.DataProcess as DP
import numpy as np
import logging
import click
from rich.logging import RichHandler
from numpy.linalg import inv

logging.basicConfig(
        level="DEBUG",
        format="%(message)s",
        datefmt="[%X]",
        handlers=[RichHandler(rich_tracebacks=True)],
    )

log = logging.getLogger(__name__)

@click.command()
@click.option("--data-dir", "-d", default="PA1/Data", help="Input data directory")
@click.option("--output-dir", "-o", default="PA1/output", help="Output directory")
@click.option("--name", "-n", default="pa1-debug-a", help="Name of the output file")
def main(data_dir, output_dir, name):
    data_dir = Path(data_dir).expanduser()
    output_dir = Path(output_dir).expanduser()
    if not output_dir.exists():
        output_dir.mkdir()

    cal_body_path = data_dir / f"{name}-calbody.txt"
    cal_body,cal_body_info = DP.load_txt_data(cal_body_path)

    cal_read_path = data_dir / f"{name}-calreadings.txt"
    cal_read,cal_read_info = DP.load_txt_data(cal_read_path)

    ND = int(cal_body_info[0])
    NA = int(cal_body_info[1])
    # Register about EM Trackers
    d = cal_body[0:ND,:].T
    D = cal_read[0:ND,:].T
    F_d = regist_matched_points(d,D)

    # log.info(f"F_d = {F_d}")

    # Register about Optical Trackers
    a = cal_body[ND:ND+NA,:].T
    A = cal_read[ND:ND+NA,:].T
    F_a = regist_matched_points(a,A)

    # log.info(f"F_a = {F_a}")

    # Find expected C
    c = cal_body[ND+NA:,:].T
    c = np.vstack((c,np.ones((1,np.size(c,axis=1)))))
    C_expected = inv(F_d) @ F_a @ c
    
    log.info(f"Result is found!\nc_expected = \n{C_expected[0:3,:].T}")
    # cal_test = [F,F,F]
    # p_t, p_post = calib_pivot_points(cal_test)

if __name__ == "__main__":
    main()



