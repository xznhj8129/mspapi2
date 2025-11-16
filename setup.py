from pathlib import Path

from setuptools import find_packages, setup

here = Path(__file__).parent
readme = (here / "README.md").read_text(encoding="utf-8")

setup(
    name="mspapi2",
    version="0.1.0",
    description="MultiWii Serial Protocol API, codec, and multi-client server",
    long_description=readme,
    long_description_content_type="text/markdown",
    url="https://github.com/xznhj8129/mspapi2",
    license="GPL-3.0-or-later",
    author="xznhj8129",
    packages=find_packages(where="mspapi2"),
    package_dir={"": "mspapi2"},
    include_package_data=True,
    package_data={"mspapi2.lib": ["msp_messages.json"]},
    python_requires=">=3.9",
    install_requires=["pyserial"],
    entry_points={
        "console_scripts": [
            "mspapi2-server=mspapi2.msp_server:main",
        ],
    },
    classifiers=[
        "License :: OSI Approved :: GNU General Public License v3 (GPLv3)",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Topic :: Software Development :: Libraries",
        "Topic :: System :: Monitoring",
    ],
)
