# Isaac SIM Jetson HIL Course Doc

Document generation status: [![example workflow](https://github.com/NVIDIA-AI-IOT/isaac-sim-jetson-hil-course-doc/actions/workflows/ci.yml/badge.svg)](https://github.com/NVIDIA-AI-IOT/isaac-sim-jetson-hil-course-doc/actions)

# About this repo

This repo is to host the course documentation for Isaac Sim Jetson HIL hands-on course.

The auto generated documentation is hosted on the following, using their CI/CD feature to automatically generate/update the HTML documentation site upon new commit:
  - (Internal) [GitLab Pages site](https://jetson.gitlab-master-pages.nvidia.com/isaac-sim-jetson-hil-course-doc/)
  - (**Not yet published**) GitHub Pages site

## How to use this repo locally

### MkDocs: Initial setup

https://squidfunk.github.io/mkdocs-material/getting-started/

```bash
sudo apt install -y docker.io
sudo docker pull squidfunk/mkdocs-material
```

### Mkdocs: Start development server on http://localhost:8000

```bash
docker run --rm -it -p 8000:8000 -v ${PWD}:/docs squidfunk/mkdocs-material
```

> If you get "docker: Got permission denied while trying to connect to the Docker daemon socket at ..." error, 
> issue `sudo chmod 666 /var/run/docker.sock` to get around with the issue.

