import os

import git
from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup

GIT_ROOT = git.Repo(
    os.path.realpath(__file__), search_parent_directories=True
).git.rev_parse("--show-toplevel")

INCLUDE_DIRS = list(
    map(
        lambda x: GIT_ROOT + "/" + x,
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
    name="controller-rpc",
    ext_modules=[
        Pybind11Extension(
            name="controller_rpc",
            sources=["order.cpp"],
            extra_compile_args=["-std=c++17", "-ggdb"],
            include_dirs=INCLUDE_DIRS,
        )
    ],
)
