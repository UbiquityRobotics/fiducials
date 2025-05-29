from setuptools import setup

package_name = 'marker_generation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    package_dir={'': 'scripts'},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Example: 'generate_markers = marker_generation.generate:main',
        ],
    },
)
