# Dylib

[![version](https://img.shields.io/badge/Version-2.2.1-blue.svg)](https://github.com/martin-olivier/Dylib/releases/tag/v2.2.1)
[![license](https://img.shields.io/badge/License-MIT-orange.svg)](https://github.com/martin-olivier/Dylib/blob/main/LICENSE)
[![cpp](https://img.shields.io/badge/Compatibility-C++11-darkgreen.svg)](https://isocpp.org)

[![ci](https://github.com/martin-olivier/Dylib/actions/workflows/CI.yml/badge.svg)](https://github.com/martin-olivier/Dylib/actions/workflows/CI.yml)
[![coverage](https://codecov.io/gh/martin-olivier/Dylib/branch/main/graph/badge.svg)](https://codecov.io/gh/martin-olivier/Dylib)

The goal of this C++ library is to load dynamic libraries (.so, .dll, .Dylib) and access its functions and global variables at runtime.  

`⭐ Don't forget to put a star if you like the project!`

## Compatibility

Works on `Linux`, `Windows`, `MacOS`

## Installation

You can fetch `Dylib` to your project using `CMake`:

```cmake
include(FetchContent)

FetchContent_Declare(
    Dylib
    GIT_REPOSITORY "https://github.com/martin-olivier/Dylib"
    GIT_TAG        "v2.2.1"
)

FetchContent_MakeAvailable(Dylib)
```

You can also click [HERE](https://github.com/martin-olivier/Dylib/releases/download/v2.2.1/Dylib.hpp) to download the `Dylib` header file.

## Documentation

### Constructor

The `Dylib` class can load a dynamic library from the system library path

```c++
// Load "foo" library from the system library path

Dylib lib("foo");
```

The `Dylib` class can also load a dynamic library from a specific path

```c++
// Load "foo" library from relative path "./libs"

Dylib lib("./libs", "foo");

// Load "foo" library from full path "/usr/lib"

Dylib lib("/usr/lib", "foo");
```

The `Dylib` class will automatically add the filename decorations of the current os to the library name, but you can disable that by setting `decorations` parameter to `Dylib::no_filename_decorations`

```c++
// Windows -> "foo.dll"
// MacOS   -> "libfoo.Dylib"
// Linux   -> "libfoo.so"

Dylib lib("foo");

// Windows -> "foo.lib"
// MacOS   -> "foo.lib"
// Linux   -> "foo.lib"

Dylib lib("foo.lib", Dylib::no_filename_decorations);
```

### Get a function or a variable

`get_function`  
Get a function from the dynamic library currently loaded in the object  

`get_variable`  
Get a global variable from the dynamic library currently loaded in the object

```c++
// Load "foo" dynamic library

Dylib lib("foo");

// Get the function "adder" (get_function<T> will return T*)

auto adder = lib.get_function<double(double, double)>("adder");

// Get the variable "pi_value" (get_variable<T> will return T&)

double pi = lib.get_variable<double>("pi_value");

// Use the function "adder" with "pi_value"

double result = adder(pi, pi);
```

### Miscellaneous tools

`has_symbol`  
Returns true if the symbol passed as parameter exists in the dynamic library, false otherwise

`get_symbol`  
Get a symbol from the dynamic library currently loaded in the object  

`native_handle`  
Returns the dynamic library handle

```c++
Dylib lib("foo");

if (lib.has_symbol("GetModule") == false)
    std::cerr << "symbol 'GetModule' not found in 'foo' lib" << std::endl;

Dylib::native_handle_type handle = lib.native_handle();
Dylib::native_symbol_type symbol = lib.get_symbol("GetModule");

assert(handle != nullptr && symbol != nullptr);
assert(symbol == dlsym(handle, "GetModule"));
```

### Exceptions

`load_error`  
This exception is raised when the library failed to load or the library encountered symbol resolution issues  

`symbol_error`  
This exception is raised when the library failed to load a symbol  

Those exceptions inherit from `Dylib::exception`

```c++
try {
    Dylib lib("foo");
    double pi_value = lib.get_variable<double>("pi_value");
    std::cout << pi_value << std::endl;
} catch (const Dylib::load_error &) {
    std::cerr << "failed to load 'foo' library" << std::endl;
} catch (const Dylib::symbol_error &) {
    std::cerr << "failed to get 'pi_value' symbol" << std::endl;
}
```

## Example

A full example about the usage of the `Dylib` library is available [HERE](example)

## Tests

To build unit tests, enter the following commands:

```sh
cmake . -B build -DDYLIB_BUILD_TESTS=ON
cmake --build build
```

To run unit tests, enter the following command inside `build` directory:

```sh
ctest
```

## Community

If you have any question about the usage of the library, do not hesitate to open a [discussion](https://github.com/martin-olivier/Dylib/discussions)

If you want to report a bug or provide a feature, do not hesitate to open an [issue](https://github.com/martin-olivier/Dylib/issues) or submit a [pull request](https://github.com/martin-olivier/Dylib/pulls)

## Contributing

Set the cmake flag `DYLIB_BUILD_TESTS` to `ON` to enable tests and make it easier for you to contribute

```sh
cmake . -B build -DDYLIB_BUILD_TESTS=ON
```

> Do not forget to sign your commits and use [conventional commits](https://www.conventionalcommits.org/en/v1.0.0/) when providing a pull request

```sh
git commit -s -m "feat: ..."
```
