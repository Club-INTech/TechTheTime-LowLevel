import os

from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup

INCLUDE_DIRS = list(
    map(
        lambda x: os.environ["VIRTUAL_ENV"] + "/../" + x,
        [
            "TechTheTime-Shared",
            "lib/Key-To-Order/include",
            "lib/Unpadded/include",
            "lib/config/include",
            "lib/static_assert/include",
            "lib/mp11/include",
            "lib/type_traits/include",
        ],
    )
)

print(INCLUDE_DIRS)

setup(
    name="controller-order",
    ext_modules=[
        Pybind11Extension(
            name="controller_order",
            sources=["order.cpp"],
            extra_compile_args=["-std=c++17", "-ggdb"],
            include_dirs=INCLUDE_DIRS,
        )
    ],
)
