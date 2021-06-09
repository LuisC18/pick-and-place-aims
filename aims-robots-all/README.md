# AIMS Lab: All Robot & EEF Support Packages

[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)

Standard set of metapackages to support AIMS Lab projects.


## Suggested Usage

Utilize this metapackage set as a starting point for AIMS lab projects. The goal of this repository is to contain all robot support configuration packages for each of the AIMS (or CDME) robots commonly used. Additionally, common end-effector support packages will be included.


### How To Use

Create your new project workspace..
`mkdir -p ~/ws_project/src`

Clone this common set of packages into your project's local workspace/src..
`git clone https://github.com/osu-aims/aims-robots-all.git ~/ws_project/src`

Remove the .GIT connection
`rm -r ~/ws_project/src/.git`

Remove packages which will not be utilized for your project..
`rm -r <unneeded packages>`


## Updating Metapackages

As this is a combined set of metapackages, the links to the source repositories have been broken. Updates to all packages will need to manually performed by rebasing updated packages into this repository. All major package additions should be wrapped into a Pull Request for proper documentation. Minor package updates may be committed directly.
