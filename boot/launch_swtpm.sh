#!/bin/bash

SERVER_PORT=2321
CTRL_PORT=2322

#> SWTPM socket interface startup
mkdir /tmp/myvtpm
swtpm socket --tpmstate dir=/tmp/myvtpm --tpm2 --ctrl type=tcp,port=$CTRL_PORT --server type=tcp,port=$SERVER_PORT --flags not-need-init
