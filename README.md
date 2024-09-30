# Parallel Forward Kinematics

A simple C++ program to perform forward kinematics calculations on the Franka Emika Panda robot using RBDL (Rigid Body Dynamics Library).

## Features

- Loads the Panda robot model from a URDF file.
- Performs forward kinematics calculations.
- Command-line options:
  - `-u`: Specify URDF file path.
  - `-n`: Set number of iterations (default: 10000).
  - `-t`: Set number of threads (default: 1).
  - `-v`: Enable verbose output.
  - `-h`: Display help message.

## Requirements

- **C++ Compiler** with C++11 support.
- **CMake** version 3.0 or higher.
- **RBDL** with URDFReader addon.
- **Eigen3** library.

## Build Instructions

1. **Clone or download** this repository.

2. **Navigate** to the project directory:

   ```bash
   cd parallel_forward_kinematics
   ```

3. **Create a build directory**:

   ```bash
   mkdir build && cd build
   ```

4. **Run CMake**:

   ```bash
   cmake ..
   ```

5. **Build the project**:

   ```bash
   make
   ```

## Usage

Run the program with optional command-line arguments:

```bash
./para_fk [options]
```

### Options

- `-u <file>`: Specify the URDF file path.
- `-n <number>`: Set the number of iterations. (default : 10000)
- `-t <number>`: Set the number of threads. (default : 1)
- `-v`: Enable verbose output.
- `-h`: Show help message.

### Example

```bash
./para_fk -u panda.urdf -n 5000 -t 5 -v
```

---

Ensure that the `panda.urdf` file is available in the specified location or provide the correct path using the `-u` option.