# gwnb_power
Battery and EC driver for phytium platform notebooks

## How to build
```bash
make
sudo make install
```

## dkms build
```bash
sudo cp gwnb_power /usr/src/gwnb_power-0.1/
sudo dkms add -m gwnb_power
sudo dkms build -m gwnb_power -v 0.1
sudo dkms install -m gwnb_power -v 0.1
```
