import setuptools

with open("README.md", "r", newline="", encoding="utf-8") as fh:
    long_description = fh.read()

setuptools.setup(
    name="pareto_leg",
    version="0.0.1",
    author="Burden Crew",
    author_email="",
    description="control software for pareto leg",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/sburden/pareto_leg",
    license="MIT",
    keywords= ['odrive', 'linkage'],
    packages=setuptools.find_packages(),
    classifiers=[
        "Development Status :: 4 - Beta",
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: POSIX :: Linux",
    ],
    python_requires='>=3.6',
    install_requires=[]
)
