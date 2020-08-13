from distutils.core import setup

setup(
    name="cc3200tool",
    version="0.1.1",
    description="A tool for CC32xx TI micro firmware update",
    author="Kiril Zyapkov",
    author_email="k.zyapkov@allterco.com",
    url="http://github.com/AlexLisnitski/cc3200tool",
    packages=['cc3200tool'],
    package_data={'cc3200tool': ['dll/*.dll', 'dll/gen2/*.ptc']},
    entry_points = {
        'console_scripts': ['cc3200tool=cc3200tool.cc:main'],
    },
    install_requires=['pyserial'],
)
