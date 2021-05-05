#!/bin/bash

cd ./beam_matching/

./beam_matching_gicp_tests
./beam_matching_icp_tests
./beam_matching_ndt_tests
./beam_matching_teaserpp_tests
./beam_matching_multi_matcher_tests
./gtests/beam_matching_loam_gtests
