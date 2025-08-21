# Tesla CAN Decoding Tools

This repository collects various tools and code for decoding and using Tesla Controller Area Network (CAN) frames. It’s a home for scripts, utilities, and references to help capture logs, interpret frames/signals, and prototype integrations that consume Tesla CAN data.

## What’s Inside

- Tools and scripts: capture, parse, filter, and transform CAN logs.
- Decoding helpers: map raw frames to human‑readable signals (DBC‑style workflows).
- References and notes: signal findings, conventions, and external resources.
- Examples: small demos showing decoding pipelines and data export/visualization.

## Safety, Legality, and Ethics

- Use responsibly: do not modify, interfere with, or distract from safety‑critical vehicle systems.
- Comply with local laws, terms, and regulations when interfacing with your vehicle.
- Remove sensitive information (e.g., VINs, GPS traces tied to home/work) from shared logs.
- Not affiliated with or endorsed by Tesla, Inc. Use at your own risk.

## Contributing

Contributions are welcome. If you add new tools or findings, include a short README in that folder and, when possible, sample data or instructions to reproduce results without proprietary information.

## DBC References and Credit

Work on decoding Tesla CAN signals draws heavily from community efforts. DBC definitions and decoding logic in this repo reference and build upon the following:

- [Josh Wardell — Model 3 DBC (Model3CAN.dbc)](https://github.com/joshwardell/model3dbc/blob/master/Model3CAN.dbc)
- [MatthewKuKanich — CAN_Commander (tesla_can.dbc)](https://github.com/MatthewKuKanich/CAN_Commander/blob/main/CAN_Resources/DBC_Files/tesla_can.dbc)
- [MatthewKuKanich — Tesla repo](https://github.com/MatthewKuKanich/Tesla)
- [comma.ai — openDBC (tesla_can.dbc)](https://github.com/commaai/opendbc/blob/master/opendbc/dbc/tesla_can.dbc)
- [OVMS — CAN-RE-Tool (Tesla models rules)](https://github.com/openvehicles/CAN-RE-Tool/blob/master/rules/teslamodels)
