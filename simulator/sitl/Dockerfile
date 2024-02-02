# Perform multistage build to avoid a pile of ardupilot deps
#FROM ardupilot/ardupilot-dev-chibios as ardupilot_builder

FROM ardupilot/ardupilot-dev-base

ARG VEHICLE=plane
ARG BRANCH=ArduPlane-stable

# Fetch ArduPilot
RUN git clone --recurse-submodules --depth 1 --shallow-submodules -j4 \
    -b ${BRANCH} https://github.com/ArduPilot/ardupilot /src/ardupilot

# Build the SITL
WORKDIR /src/ardupilot
RUN ./waf configure && ./waf ${VEHICLE}

#

ARG VEHICLE=plane


#get JSBSim
RUN git clone https://github.com/JSBSim-Team/jsbsim.git /src/jsbsim
RUN mkdir /src/jsbsim/build
WORKDIR /src/jsbsim/build
RUN cmake -DCMAKE_CXX_FLAGS_RELEASE="-O3 -march=native -mtune=native" \
          -DCMAKE_C_FLAGS_RELEASE="-O3 -march=native -mtune=native" \
        -DCMAKE_BUILD_TYPE=Release ..
RUN make -j2

ENV PATH="$PATH:/src/jsbsim/build/src"

WORKDIR /home/root

COPY entrypoint.sh \
    lookup_parameter_files.py \
    /home/root/

# The VEHICLE environment variable should not be overridden as
#  the build will not exist for it
ENV AP_VEHICLE ${VEHICLE}
ENV AP_SYSID 1
ENV AP_SYSID_BASE 1
ENV AP_PARAM_PATH ""
ENV AP_PARAM_FILES ""
ENV AP_HOME 51.4235413,-2.6708488,50,250
ENV AP_OFFSET_X 0
ENV AP_OFFSET_Y 0
# AP_DISTRIBUTE
ENV AP_USE_JSBSIM 1
ENV AP_SIM_ADDRESS 127.0.0.1
# AP_SIM_HOST

ENTRYPOINT [ "/home/root/entrypoint.sh" ]