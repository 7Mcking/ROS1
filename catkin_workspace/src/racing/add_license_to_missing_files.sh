#!/bin/bash
read -r -d '' license <<-"EOF"
/*    Modified by Institute for Automotive Engineering (ika), RWTH University
 *    All rigths reserved
 *
 *    This file is part of "Self-Driving Lab I - Software Framework".
 *    Redistribution and use in source and binary forms, with or without
 *    modification, are permitted exclusively under the terms defined in
 *    the licese file. You should have received a copy of the license with
 *    this file. If not, please visit:
 *    https://git.rwth-aachen.de/ika/sdl1-ws2019.
 */
EOF

files=$(grep -rL "Copyright (c) 2019, Institute for Automotive Engineering (ika), RWTH University" * | grep "\.h\|\.cpp")

for f in $files
do
  echo -e "$license" > temp  
  cat $f >> temp
  mv temp $f
done
