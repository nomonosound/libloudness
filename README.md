# libloudness
A library that implements the ITU BS.1770 loudness algorithm and true peak recommendations for use with e.g. EBU R-128

Originally a c++ port of [libebur128](https://github.com/jiixyj/libebur128), with various optimizations and convenient extensions

## Features
- Written in c++20
- Supports [EBU R 128](https://tech.ebu.ch/docs/r/r128.pdf)
  - M, S and I modes ([EBU Tech 3341](https://tech.ebu.ch/docs/tech/tech3341.pdf)
  - Loudness range measurements ([EBU Tech 3342](https://tech.ebu.ch/docs/tech/tech3342.pdf))
  - True peak scanner
- Additional features based on ITU BS.1770
  - Median loudness calculation
  - Calculate global loudness without the relative gating
- Supports all realistic samplerates
- Supports both interleaved and non-interleaved input
- Limited optional multithreading support

## Installation

## Usage
