# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Installation script for the 'irim_indy7' python package."""

from setuptools import setup, find_packages

# Installation operation
setup(
    name="irim_indy7",
    author="Isaac Lab Project Developers",
    version="0.1.0",
    description="Indy7 robot tasks for Isaac Lab",
    include_package_data=True,
    python_requires=">=3.10",
    packages=find_packages(),
    zip_safe=False,
)