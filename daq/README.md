The "DAQ" is a separate device for measuring current.
It is installed in series into the power supply of the source (ODrive board in this case) to be measured.
Data is sampled at 5[KHz] and streamed in real time to a python script, which aggregates the data and writes it to a CSV file.

The DAQ is just a 1bitsy and a [ACHS-7124](https://www.pololu.com/product/4033)an analog in-series current measurement sensor.
A voltage divider that maps the input from 0-to-5 to 0-to-3.3 is also present.

## Usage

To start collecting data, use the **acquire\_data.py** script.
````
./acquire_data.py --seconds=3
````
