# Boot Phase Securiy

Purpose: Add SWTPM PCR hashing to the boot chain.

## Limitation

Any path containing spaces will not be hashed correctly.

## Methodology

1. Add to boot chain: https://belongtothenight.github.io/HOW_TO/how_to_LINUX-auto_launch_program_on_boot/
2. Extend PCR: Add log entry when extending, to PCR x.

## Reference

1. https://uapi-group.org/specifications/specs/linux_tpm_pcr_registry/
2. https://security.stackexchange.com/questions/258649/usage-of-tpm-pcrs-in-linux
3. https://trustedcomputinggroup.org/wp-content/uploads/TCG_PCClient_PFP_r1p05_v23_pub.pdf Section 3.3.4, Section 8.2
4. https://stackoverflow.com/questions/75916117/go-lang-tpm2-library-pcr-extend-pcr-read-inconsistency
5. https://stackoverflow.com/questions/36718019/journalctl-remove-logs-of-a-specific-unit
