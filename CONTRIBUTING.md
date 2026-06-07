# Contributing to Exact2D CAD

First off, thank you for considering contributing to Exact2D CAD! It's people like you that make open-source tools great. 

Exact2D CAD relies on a complex exact algebraic geometry kernel, and contributions of all kinds are welcome—whether you are fixing a bug, adding support for a new IO format (like enhancements to DXF or SVG parsing), improving the documentation, or proposing new features.

This document provides guidelines and workflows for contributing to the project.

## Table of Contents

* [Code of Conduct](#code-of-conduct)
* [Getting Started](#getting-started)
* [Development Workflow](#development-workflow)
* [Submitting a Pull Request](#submitting-a-pull-request)
* [Reporting Bugs](#reporting-bugs)
* [Suggesting Enhancements](#suggesting-enhancements)

## Code of Conduct

By participating in this project, you are expected to uphold our [Code of Conduct](CODE_OF_CONDUCT.md). Please report unacceptable behavior to the email address provided in that document.

## Getting Started

### Prerequisites

Exact2D CAD is written in Rust. You will need the latest stable version of the Rust toolchain installed. 

We recommend using [rustup](https://rustup.rs/) to install and manage your Rust toolchain:

```bash
curl --proto '=https' --tlsv1.2 -sSf [https://sh.rustup.rs](https://sh.rustup.rs) | sh
