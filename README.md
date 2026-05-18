# Water Pipeline Leak Detection Network

This project simulates a LoRa-based water pipeline leak detection system in OMNeT++ using INET and FLoRa. FLoRa is a LoRa simulation framework based on OMNeT++ and INET, and OMNeT++ simulations are configured through files such as `omnetpp.ini`. :contentReference[oaicite:0]{index=0}

## Overview

Sensor nodes are placed along a pipeline and send regular status packets and random leak alarm packets to a LoRa gateway. The gateway forwards packets to the monitoring server. Leak generation follows a Poisson process with exponential inter-arrival times.

## Requirements

- OMNeT++
- INET Framework
- FLoRa Framework
- Supported C++ compiler

## How to Run

1. Open OMNeT++ IDE.
2. Import INET and FLoRa.
3. Put all the files inside the Flora; if there are any with the same name, keep our files.
4. Select omnetpp.ini, right click it, and hover your mouse on the Run as button, select omnet.
5. Run the simulation.

## Output Metrics

The simulation records sent packets, leak alarms, leak inter-arrival times, packet reception counts, throughput, spreading factor, transmission power, and ADR-related values.

## Contributors

- Tuna Efe Yavuz
- Ahmet Kürşat Ağır
