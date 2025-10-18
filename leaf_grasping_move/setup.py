from setuptools import setup

package_name = "leaf_grasping_move"

setup(
    name=package_name,
    version="0.0.0",
    packages=[],
    py_modules=["test.test_publisher"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="your_name",
    maintainer_email="your_email@example.com",
    description="C++ arm control package with Python tests",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "test_publisher = test.test_publisher:main",
        ],
    },
)
