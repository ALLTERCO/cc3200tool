from distutils.core import setup

setup(
    name="cc3200tool",
    version="0.1.0",
    description="A tool to upload files to TI CC3200",
    author="Kiril Zyapkov",
    author_email="k.zyapkov@allterco.com",
    url="http://github.com/allterco/cc3200tool",
    packages=['cc3200tool'],
    package_data={'cc3200tool': ['dll/*.dll']},
    scripts=['scripts/cc3200tool'],
    install_requires=['pyserial'],
)
