# Potential commands to use TPM2.0

## Considered Commands

1. read PCR all values
```bash
tpm2_pcrread sha256
```
2. read PCR 0 value
```bash
tpm2_pcrread sha256:0
```
3. initialize PCR 0, requires 20 bytes of data, 8 x 8 x hex values, at max can put 5 pcrs in one command
```bash
tpm2_pcrextend 0:sha256=123456789abcdef0000000000000000000000000000000000000000000000000
```
4. hash a file (-t is optional)
```bash
tpm2_hash -C o -g sha256 -o hash.out -t ticketfile file.in
```
5. export the hash to hex string and store in file
```bash
xxd -p hash.out | tr -d '\n' > hash.hex
```
6. hash a file and extend the pcr ?
```bash
tpm2_pcrextend 0:sha256=$(cat hash.hex)
```
7. combine 4, 5, 6 in one command
```bash
tpm2_pcrextend 0:sha256=$(tpm2_hash -C o -g sha256 file.in | xxd -p | tr -d '\n')
```

## Measured Boot Chain Process v1 (inalid)

1. decide PCR index to extend, this depends on how the PCR is used after IBMACS is setup
2. read the PCR value and make sure it is 0 before extending
3. read the previous PCR value
4. concatenate measuring software files and previous PCR value into a single file
5. hash the concatenated file
6. extend the PCR with the hash value
7. IBMACS should compare the final PCR value with previous boot PCR value, if they are same, then the boot process is measured correctly and nothing is changed.

This method doesn't work because tpm2_pcrextend will modify the PCR index location based on its current holding value, so even if the hash value is the same, the PCR index value will still change

## Measured Boot Chain Process v2

Let selected PCR index be x

1. decide PCR index to extend, this depends on how the PCR is used after IBMACS is setup
2. in initial setup process, extend the first PCR index x with 0x0 value with concatenated file of measuring software files and previous PCR value x-1, tpm2_pcrextend should only be used in this case
3. in future boot process, tpm2_pcrread previous PCR index x-1 value and current PCR index x value, concatenate the measuring software files and previou PCR index x-1 value, hash them, see wether it is the same as PCR index x value, if not, extend PCR index x value, modify the PCR location, showing things have changed

## Observations During Testing

1. Initially on boot (fist OS boot, IBMACS not installed), PCR 0-16 are 0x0, PCR 17-22 are 0xF, PCR 23 is 0x0
2. tpm2_pcrextend will extend the PCR index location you specified and modify the same PCR index location
3. After OS reboot, PCR table won't be cleaned
4. Using SHA-1 or SHA-256 will store value in different PCR table
5. tpm2_pcrreset can not be used to reset the PCR values.
6. To fully reset both SHA-1 and SHA-256 PCR tables, you need to press the reset button on the TPM chip, and reboot your system.
7. TPM hardware reset basically acts as a on-board memory reset, so all the values stored in the TPM chip will be lost.
