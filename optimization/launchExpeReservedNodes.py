#!/usr/bin/python3

#  The cluster that we use in IRIDIA
#
import subprocess
import os
import argparse
from argparse import ArgumentParser, ArgumentTypeError
import re
import random

#starting seed:
SEED = 123456

def parseNumList(string):
    m = re.match(r'(\d+)(?:-(\d+))?$', string)
    # ^ (or use .split('-'). anyway you like.)
    if not m:
        raise ArgumentTypeError("'" + string + "' is not a range of number. Expected forms like '0-5' or '2'.")
    start = m.group(1)
    end = m.group(2) or start
    return list(range(int(start,10), int(end,10)+1))


def main(p):
    args = p.parse_args()
    os.environ["OMPI_MCA_plm_rsh_disable_qrsh"] = "1"

    slaves = args.parallel
    sce = args.scenario
    seed = SEED

    for i in args.nbiterations:
        seed = seed+random.randint(1,10)
        execdir = "{}-{:02d}".format(args.execdir,i)
        print("Starting new iteration.. N={},D={},S={}..".format(i,execdir,seed))
        try:
            os.mkdir(execdir)
        except OSError:
            print("Exec folder already exists, continuing..")
            continue

        subprocess.check_call("""/opt/openmpi/bin/mpirun -x
            OMPI_MCA_plm_rsh_disable_qrsh -np 1
            irace --exec-dir={execdir} --parallel {nb_slaves} --seed {seed}
            --mpi 1 -s {scenario}""".format(\
            execdir=execdir, nb_slaves=slaves, seed=seed, scenario=sce), shell=True)

        #/opt/openmpi/bin/mpirun -x OMPI_MCA_plm_rsh_disable_qrsh -np 1 irace --exec-dir=$EXECDIR --parallel $NB_SLAVES --seed $RUNSEED --mpi 1 -s scenario_desi_nogian.txt


if __name__ == '__main__':
    p = argparse.ArgumentParser(description='runs IRACE on the cluster for reserved nodes')
    p.add_argument('-s', '--scenario', help='the irace scenario to run', required=True)
    p.add_argument('-n', '--nbiterations', type=parseNumList, help='nb of irace runs to perform (can also be a range)', required=True)
    p.add_argument('-d', '--dir', help='exec dir of irace ("-NN" number is added)', required=True)
    p.add_argument('-p', '--parallel', help='number of paralell CPUs to use (total p+1)', required=True)
    main(p)
