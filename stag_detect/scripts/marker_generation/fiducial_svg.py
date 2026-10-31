#! /usr/bin/python
import em
from urllib.parse import quote
import base64


paper_size = (179, 179)  # Cropped Aruco Marker
# Must not contain a white space before the first tag!

def genFiducialwithoutBorder(upper, lower, tag, pack_id, marker_path_in, fid_len=140.0, year=2021):
    with open(marker_path_in, "rb") as f:
        encoded = base64.b64encode(f.read()).decode("ascii")

    marker_path = "data:image/png;base64," + encoded
    if tag == "GO":  # Green
        return em.expand("""<?xml version="1.0" encoding="UTF-8" standalone="no"?>
<svg
   xmlns:dc="http://purl.org/dc/elements/1.1/"
   xmlns:cc="http://creativecommons.org/ns#"
   xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#"
   xmlns:svg="http://www.w3.org/2000/svg"
   xmlns="http://www.w3.org/2000/svg"
   xmlns:sodipodi="http://sodipodi.sourceforge.net/DTD/sodipodi-0.dtd"
   xmlns:inkscape="http://www.inkscape.org/namespaces/inkscape"
   viewBox="0 0 1077.1653 1266.1417"
   version="1.1"
   id="svg28"
   sodipodi:docname="go.svg"
   inkscape:version="0.92.4 (5da689c313, 2019-01-14)"
   width="28.5cm"
   height="33.5cm">
  <metadata
     id="metadata34">
    <rdf:RDF>
      <cc:Work
         rdf:about="">
        <dc:format>image/svg+xml</dc:format>
        <dc:type
           rdf:resource="http://purl.org/dc/dcmitype/StillImage" />
        <dc:title>fiducial-markers</dc:title>
      </cc:Work>
    </rdf:RDF>
  </metadata>
  <defs
     id="defs32" />
  <sodipodi:namedview
     pagecolor="#ffffff"
     bordercolor="#666666"
     borderopacity="1"
     objecttolerance="10"
     gridtolerance="10"
     guidetolerance="10"
     inkscape:pageopacity="0"
     inkscape:pageshadow="2"
     inkscape:window-width="1920"
     inkscape:window-height="996"
     id="namedview30"
     showgrid="false"
     inkscape:zoom="0.45254834"
     inkscape:cx="629.33114"
     inkscape:cy="723.29213"
     inkscape:window-x="-8"
     inkscape:window-y="-8"
     inkscape:window-maximized="1"
     inkscape:current-layer="g838"
     units="cm" />
  <title
     id="title2">fiducial-markers</title>
  <g
     id="g838"
     transform="translate(0,261842.8)">
    <g
       id="g859">
      <path
         id="polygon4"
         d="m 538.58265,-261834.81 -359.05273,255.82 v 861.11 h 179.52539 172.74805 v -6.43 h 13.55859 v 6.43 H 718.11 897.63534 v -861.11 z"
         style="fill:#57c15e;fill-opacity:1;stroke-width:0.27037317"
         inkscape:connector-curvature="0" />
      <path
         sodipodi:nodetypes="cccc"
         inkscape:connector-curvature="0"
         id="polygon12"
         d="m 629.94292,-261729.76 -91.35861,-66.72 -91.36192,66.72 z"
         style="fill:#ffffff;stroke-width:0.2904962;fill-opacity:1" />
      <g
         transform="translate(-209.54712,-261949.52)"
         id="g1061">
        <g
           transform="translate(-38.628653)"
           id="g930">
          <g
             transform="translate(41.339389)"
             id="g843">



            <g  transform="translate(-155 25)">
               <path
                  d="m 611.69999,1085.6544 v 0.2805 c 5.95847,1.3249 12.10882,4.2589 15.60754,8.5161 v 32.2649 c -5.10781,5.1095 -12.48894,7.8535 -18.06731,8.7042 3.87881,2.2715 7.09514,4.7312 9.55491,7.4752 2.0797,2.2716 4.82364,5.9622 8.13412,11.07 -0.94661,8.2318 -3.31048,15.1387 -10.02734,18.5451 l -11.44461,-17.1243 c -4.63537,-6.9087 -7.75758,-11.2599 -9.46079,-13.0572 -1.70321,-1.7974 -3.49871,-3.0282 -5.39198,-3.6907 -1.79732,-0.6624 -4.63359,-0.9466 -8.41828,-1.0407 v -14.194 h 9.08073 q 12.91249,0 16.17592,-1.1348 c 2.17199,-0.7565 3.7847,-1.991 5.01364,-3.7847 1.22899,-1.7937 1.79733,-3.9729 1.79733,-6.7168 0,-3.0282 -0.85067,-5.3938 -2.45977,-7.1912 -1.60727,-1.8932 -3.78287,-3.0281 -6.71508,-3.5023 -1.419,-0.181 -5.86619,-0.3783 -13.14955,-0.3783 h -9.74322 v -15.041 h -23.1679 v 53.1721 c 0,12.869 0.75661,22.5199 2.26976,29.1409 q 1.562,6.6716 6.52681,12.7731 c 3.31048,3.982 7.75763,7.287 13.33783,9.8409 5.67614,2.4597 14.00032,3.7847 24.97788,3.7847 9.08256,0 16.46008,-1.1349 22.04028,-3.4064 5.58203,-2.3657 10.12329,-5.488 13.43377,-9.3667 3.40459,-3.8788 5.6743,-8.7043 6.99924,-14.3822 1.23077,-5.6779 1.89143,-15.423 1.89143,-29.1409 v -52.4155 z"
                  style="fill:#ffffff;stroke-width:0.18099914"
                  id="path10"
                  inkscape:connector-curvature="0" />
               <text
                  style="font-weight:800;font-size:13.09123993px;font-family:Arial-Black, 'Arial Black';fill:#ffffff;stroke-width:0.2233299"
                  id="text8-5-4"
                  x="653.39105"
               y="1088.5574">TM</text>
            </g>
          </g>
        </g>
      </g>
      <text
         id="text996-0"
         y="-261603.02"
         x="471.53461"
         style="font-weight:800;font-size:80px;font-family:Arial-Black, 'Arial Black';fill:#ffffff;stroke-width:1.06528783;fill-opacity:1;">GO</text>
      <path
         inkscape:connector-curvature="0"
         style="fill:#020202;fill-opacity:1;fill-rule:nonzero;stroke:none"
         d="m 274.02465,-261486.58 h 529.11609 v 529.12 H 274.02465 Z"
         id="path858" />
      <text
         id="text843"
         y="-260734.81"
         x="361.87097"
         style="font-weight:800;font-size:12.79926109px;font-family:Arial-Black, 'Arial Black';fill:#e6e6e6;stroke-width:0.21834888">&#169; @(year) Ubiquity Robotics, Inc. All Rights Reserved</text>
      <text
         y="-261693.03"
         x="487.31073"
         id="text8-5"
         style="font-weight:800;font-size:24.34896851px;font-family:Arial-Black, 'Arial Black';fill:#e6e6e6;stroke-width:0.41538101;fill-opacity:1;">Pack: @(pack_id)</text>
    </g>
  </g>

   <text
      style="font-weight:800;font-size:99.13722229px;font-family:Arial-Black, 'Arial Black';fill:#ffffff;stroke-width:0.21713541"
      id="text8"
      text-anchor="middle"
      x="50%"
      y="1070">@(num)</text>


  <g transform="translate(200,282) scale(1) translate(0, 0) scale(1) ">
                                                <rect x="@( (paper_width - fid_len)/2)mm" y="@( (paper_height - fid_len)/2)mm" width="@(fid_len)mm" height="4.0mm" style="stroke:none; fill:black"/>
                                                <rect x="@( (paper_width - fid_len)/2)mm" y="@( (paper_height + fid_len)/2 - 4)mm" width="@(fid_len)mm" height="4.0mm" style="stroke:none; fill:black"/>
                                                <rect x="@( (paper_width - fid_len)/2)mm" y="@( (paper_height - fid_len)/2)mm" width="4.0mm" height="@(fid_len)mm" style="stroke:none; fill:black"/>
                                                <rect x="@( (paper_width + fid_len)/2 - 4)mm" y="@( (paper_height - fid_len)/2)mm" width="4.0mm" height="@(fid_len)mm" style="stroke:none; fill:black"/>
                                                <image xmlns:xlink="http://www.w3.org/1999/xlink" x="@( (paper_width - fid_len)/2)mm" y="@( (paper_height - fid_len)/2)mm" width="@(fid_len)mm" height="@(fid_len)mm" xlink:href="@(marker_path)" />
                                        </g>

   <g transform = "translate(2,33) rotate(45 60 60)">
       <rect id = "myRect"
         x = "480"
         y = "0"
         height = "190"
         width = "65"
         stroke = "none"
         fill = "black" />
   </g>
                                        </svg>""", {"id": "%04d" % upper, "i": upper, "num": lower, "pack_id": pack_id, "paper_width": paper_size[0], "paper_height": paper_size[1], "fid_len": fid_len, "year": year, "marker_path": marker_path})

    elif tag == "STOP":  # Red
        return em.expand("""<?xml version="1.0" encoding="UTF-8" standalone="no"?>
<svg
   xmlns:dc="http://purl.org/dc/elements/1.1/"
   xmlns:cc="http://creativecommons.org/ns#"
   xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#"
   xmlns:svg="http://www.w3.org/2000/svg"
   xmlns="http://www.w3.org/2000/svg"
   xmlns:sodipodi="http://sodipodi.sourceforge.net/DTD/sodipodi-0.dtd"
   xmlns:inkscape="http://www.inkscape.org/namespaces/inkscape"
   viewBox="0 0 1077.1648 1266.1415"
   version="1.1"
   id="svg28"
   sodipodi:docname="stop.svg"
   inkscape:version="0.92.4 (5da689c313, 2019-01-14)"
   width="28.5cm"
   height="33.5cm">
  <metadata
     id="metadata34">
    <rdf:RDF>
      <cc:Work
         rdf:about="">
        <dc:format>image/svg+xml</dc:format>
        <dc:type
           rdf:resource="http://purl.org/dc/dcmitype/StillImage" />
        <dc:title>fiducial-markers</dc:title>
      </cc:Work>
    </rdf:RDF>
  </metadata>
  <defs
     id="defs32">
    <pattern
       y="0"
       x="0"
       height="6"
       width="6"
       patternUnits="userSpaceOnUse"
       id="EMFhbasepattern" />
  </defs>
  <sodipodi:namedview
     pagecolor="#ffffff"
     bordercolor="#666666"
     borderopacity="1"
     objecttolerance="10"
     gridtolerance="10"
     guidetolerance="10"
     inkscape:pageopacity="0"
     inkscape:pageshadow="2"
     inkscape:window-width="1920"
     inkscape:window-height="996"
     id="namedview30"
     showgrid="false"
     inkscape:zoom="0.5"
     inkscape:cx="807.10903"
     inkscape:cy="679.09173"
     inkscape:window-x="-8"
     inkscape:window-y="-8"
     inkscape:window-maximized="1"
     inkscape:current-layer="svg28"
     units="cm" />
  <title
     id="title2">fiducial-markers</title>
  <g
     id="g871">
    <path
       id="path4"
       d="M 320.83071,119.70512 12.8854,420.58794 v 425.51171 l 307.94531,300.89065 210.97266,-0.01 v -11.9297 h 13.55859 v 11.9297 l 210.96875,-0.01 307.94919,-300.89056 v -425.5 L 756.33657,119.70512 H 545.36196 v 10.3711 h -13.55859 v -10.3711 z"
       style="fill:#cb4d4d;stroke-width:0.26510242;fill-opacity:1"
       inkscape:connector-curvature="0" />
    <path
       inkscape:connector-curvature="0"
       style="fill:#020202;fill-opacity:1;fill-rule:nonzero;stroke:none"
       d="M 274.02466,356.21937 H 803.14075 V 885.33542 H 274.02466 Z"
       id="path858" />
    <text
       id="text996-0"
       y="243.28294"
       x="420.14944"
       style="font-weight:800;font-size:80px;font-family:Arial-Black, 'Arial Black';fill:#ffffff;stroke-width:1.06528783">STOP</text>
    <text
       y="158.98259"
       x="487.31076"
       id="text8-5"
       style="font-weight:800;font-size:24.34896851px;font-family:Arial-Black, 'Arial Black';fill:#cccccc;stroke-width:0.41538101">Pack: @(pack_id)</text>
    <path
       inkscape:connector-curvature="0"
       id="rect1123"
       d="m 330.05923,132.49223 v 8.94439 h 92.50585 v -8.94439 z m 324.54101,0 v 8.94439 h 97.32617 v -8.94439 z"
       style="fill:#ffffff;fill-opacity:1;stroke:none;stroke-width:0.72918367;stroke-linecap:round;stroke-linejoin:round;stroke-miterlimit:4;stroke-dasharray:none;stroke-opacity:1" />
    <g
       transform="translate(-197.95973,-127.70816)"
       id="g1242">
      <polygon
         points="2334.51,659.13 2059.24,397.68 1783.98,659.13 1938.07,659.13 1938.07,916.32 2180.42,916.32 2180.42,659.13 "
         style="fill:#ffffff"
         id="polygon6"
         transform="matrix(0.21393362,0,0,0.21393362,-138.9824,620.50149)" />
      <polygon
         points="2180.42,659.13 2334.51,659.13 2059.24,397.68 1783.98,659.13 1938.07,659.13 1938.07,916.32 2180.42,916.32 "
         style="fill:#ffffff"
         id="polygon6-6"
         transform="matrix(0.21393362,0,0,0.21393362,730.98373,620.50149)" />
    </g>
    <g
       transform="translate(-209.54711,-106.72059)"
       id="g1061">
      <g
         transform="translate(-38.628653)"
         id="g930">
        <g
           transform="translate(5.5183533)"
           id="g851">
          <g
             transform="translate(-60)"
             id="g858">
            <path
               inkscape:connector-curvature="0"
               id="path10"
               style="fill:#ffffff;stroke-width:0.18099914"
               d="m 611.69999,1085.6544 v 0.2805 c 5.95847,1.3249 12.10882,4.2589 15.60754,8.5161 v 32.2649 c -5.10781,5.1095 -12.48894,7.8535 -18.06731,8.7042 3.87881,2.2715 7.09514,4.7312 9.55491,7.4752 2.0797,2.2716 4.82364,5.9622 8.13412,11.07 -0.94661,8.2318 -3.31048,15.1387 -10.02734,18.5451 l -11.44461,-17.1243 c -4.63537,-6.9087 -7.75758,-11.2599 -9.46079,-13.0572 -1.70321,-1.7974 -3.49871,-3.0282 -5.39198,-3.6907 -1.79732,-0.6624 -4.63359,-0.9466 -8.41828,-1.0407 v -14.194 h 9.08073 q 12.91249,0 16.17592,-1.1348 c 2.17199,-0.7565 3.7847,-1.991 5.01364,-3.7847 1.22899,-1.7937 1.79733,-3.9729 1.79733,-6.7168 0,-3.0282 -0.85067,-5.3938 -2.45977,-7.1912 -1.60727,-1.8932 -3.78287,-3.0281 -6.71508,-3.5023 -1.419,-0.181 -5.86619,-0.3783 -13.14955,-0.3783 h -9.74322 v -15.041 h -23.1679 v 53.1721 c 0,12.869 0.75661,22.5199 2.26976,29.1409 q 1.562,6.6716 6.52681,12.7731 c 3.31048,3.982 7.75763,7.287 13.33783,9.8409 5.67614,2.4597 14.00032,3.7847 24.97788,3.7847 9.08256,0 16.46008,-1.1349 22.04028,-3.4064 5.58203,-2.3657 10.12329,-5.488 13.43377,-9.3667 3.40459,-3.8788 5.6743,-8.7043 6.99924,-14.3822 1.23077,-5.6779 1.89143,-15.423 1.89143,-29.1409 v -52.4155 z" />
            <text
               y="1088.5574"
               x="653.39105"
               id="text8-5-4"
               style="font-weight:800;font-size:13.09123993px;font-family:Arial-Black, 'Arial Black';fill:#ffffff;stroke-width:0.2233299">TM</text>
          </g>
        </g>
      </g>
    </g>
    <text
       id="text843"
       y="1107.9775"
       x="361.871"
       style="font-weight:800;font-size:12.79926109px;font-family:Arial-Black, 'Arial Black';fill:#e6e6e6;stroke-width:0.21834888">&#169; @(year) Ubiquity Robotics, Inc. All Rights Reserved</text>

   <text
      style="font-weight:800;font-size:99.13722229px;font-family:Arial-Black, 'Arial Black';fill:#ffffff;stroke-width:0.21713541"
      id="text8"
      text-anchor="middle"
      x="50%"
      y="1070">@(num)</text>

  </g>
                                            <g transform="translate(200,282) scale(1) translate(0, 0) scale(1) ">
                                                    <rect x="@( (paper_width - fid_len)/2)mm" y="@( (paper_height - fid_len)/2)mm" width="@(fid_len)mm" height="4.0mm" style="stroke:none; fill:black"/>
               					<rect x="@( (paper_width - fid_len)/2)mm" y="@( (paper_height + fid_len)/2 - 4)mm" width="@(fid_len)mm" height="4.0mm" style="stroke:none; fill:black"/>
                				<rect x="@( (paper_width - fid_len)/2)mm" y="@( (paper_height - fid_len)/2)mm" width="4.0mm" height="@(fid_len)mm" style="stroke:none; fill:black"/>
                				<rect x="@( (paper_width + fid_len)/2 - 4)mm" y="@( (paper_height - fid_len)/2)mm" width="4.0mm" height="@(fid_len)mm" style="stroke:none; fill:black"/>
                				<image xmlns:xlink="http://www.w3.org/1999/xlink" x="@( (paper_width - fid_len)/2)mm" y="@( (paper_height - fid_len)/2)mm" width="@(fid_len)mm" height="@(fid_len)mm" xlink:href="@(marker_path)" />
					</g>
   <g transform = "translate(2,33) rotate(45 60 60)">
       <rect id = "myRect"
         x = "480"
         y = "0"
         height = "190"
         width = "65"
         stroke = "none"
         fill = "black" />
   </g>
				</svg>""", {"id": "%04d" % upper, "i": upper, "num": lower, "pack_id": pack_id, "paper_width": paper_size[0], "paper_height": paper_size[1], "fid_len": fid_len, "year": year, "marker_path": marker_path})

    elif tag == "TURN":  # Blue
        return em.expand("""<?xml version="1.0" encoding="UTF-8" standalone="no"?>
<svg
   xmlns:dc="http://purl.org/dc/elements/1.1/"
   xmlns:cc="http://creativecommons.org/ns#"
   xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#"
   xmlns:svg="http://www.w3.org/2000/svg"
   xmlns="http://www.w3.org/2000/svg"
   xmlns:sodipodi="http://sodipodi.sourceforge.net/DTD/sodipodi-0.dtd"
   xmlns:inkscape="http://www.inkscape.org/namespaces/inkscape"
   viewBox="0 0 1077.1653 1266.1417"
   version="1.1"
   id="svg38"
   sodipodi:docname="turn.svg"
   inkscape:version="0.92.4 (5da689c313, 2019-01-14)"
   width="28.5cm"
   height="33.5cm">
  <metadata
     id="metadata44">
    <rdf:RDF>
      <cc:Work
         rdf:about="">
        <dc:format>image/svg+xml</dc:format>
        <dc:type
           rdf:resource="http://purl.org/dc/dcmitype/StillImage" />
        <dc:title>fiducial-markers</dc:title>
      </cc:Work>
    </rdf:RDF>
  </metadata>
  <defs
     id="defs42">
    <pattern
       y="0"
       x="0"
       height="6"
       width="6"
       patternUnits="userSpaceOnUse"
       id="EMFhbasepattern" />
  </defs>
  <sodipodi:namedview
     pagecolor="#ffffff"
     bordercolor="#666666"
     borderopacity="1"
     objecttolerance="10"
     gridtolerance="10"
     guidetolerance="10"
     inkscape:pageopacity="0"
     inkscape:pageshadow="2"
     inkscape:window-width="1920"
     inkscape:window-height="996"
     id="namedview40"
     showgrid="false"
     inkscape:zoom="0.5"
     inkscape:cx="679.8302"
     inkscape:cy="891.98892"
     inkscape:window-x="-8"
     inkscape:window-y="-8"
     inkscape:window-maximized="1"
     inkscape:current-layer="svg38"
     units="cm" />
  <title
     id="title2">fiducial-markers</title>
  <g
     id="g862">
    <path
       sodipodi:nodetypes="cccccccccccccccc"
       inkscape:connector-curvature="0"
       id="polygon4"
       d="m 538.58265,11.79068 -89.76318,63.95557 -89.76319,63.95557 -179.52636,127.91113 v 861.11135 h 179.52539 172.74805 v -6.7847 h 13.55859 v 6.7847 H 718.11 897.63534 V 267.61295 L 718.109,139.70182 628.34582,75.74625 Z"
       style="fill:#5391c5;stroke-width:0.27037317;fill-opacity:1" />
    <path
       sodipodi:nodetypes="cccc"
       inkscape:connector-curvature="0"
       id="polygon12"
       d="M 629.94292,116.84308 538.58431,50.12179 447.22239,116.84308 Z"
       style="fill:#ffffff;stroke-width:0.2904962" />
    <g
       transform="translate(-173.72605,-102.92214)"
       id="g1061">
      <g
         transform="translate(2e-6,-3.8007758)"
         id="g1061-0">
        <g
           transform="translate(-38.628653)"
           id="g930">
          <g
             transform="translate(5.5183183)"
             id="g857">
             <g  transform="translate(-155 25)">
            <path
               d="m 611.69999,1085.6544 v 0.2805 c 5.95847,1.3249 12.10882,4.2589 15.60754,8.5161 v 32.2649 c -5.10781,5.1095 -12.48894,7.8535 -18.06731,8.7042 3.87881,2.2715 7.09514,4.7312 9.55491,7.4752 2.0797,2.2716 4.82364,5.9622 8.13412,11.07 -0.94661,8.2318 -3.31048,15.1387 -10.02734,18.5451 l -11.44461,-17.1243 c -4.63537,-6.9087 -7.75758,-11.2599 -9.46079,-13.0572 -1.70321,-1.7974 -3.49871,-3.0282 -5.39198,-3.6907 -1.79732,-0.6624 -4.63359,-0.9466 -8.41828,-1.0407 v -14.194 h 9.08073 q 12.91249,0 16.17592,-1.1348 c 2.17199,-0.7565 3.7847,-1.991 5.01364,-3.7847 1.22899,-1.7937 1.79733,-3.9729 1.79733,-6.7168 0,-3.0282 -0.85067,-5.3938 -2.45977,-7.1912 -1.60727,-1.8932 -3.78287,-3.0281 -6.71508,-3.5023 -1.419,-0.181 -5.86619,-0.3783 -13.14955,-0.3783 h -9.74322 v -15.041 h -23.1679 v 53.1721 c 0,12.869 0.75661,22.5199 2.26976,29.1409 q 1.562,6.6716 6.52681,12.7731 c 3.31048,3.982 7.75763,7.287 13.33783,9.8409 5.67614,2.4597 14.00032,3.7847 24.97788,3.7847 9.08256,0 16.46008,-1.1349 22.04028,-3.4064 5.58203,-2.3657 10.12329,-5.488 13.43377,-9.3667 3.40459,-3.8788 5.6743,-8.7043 6.99924,-14.3822 1.23077,-5.6779 1.89143,-15.423 1.89143,-29.1409 v -52.4155 z"
               style="fill:#ffffff;stroke-width:0.18099914"
               id="path10"
               inkscape:connector-curvature="0" />
            <text
               style="font-weight:800;font-size:13.09123993px;font-family:Arial-Black, 'Arial Black';fill:#ffffff;stroke-width:0.2233299"
               id="text8-5-4"
               x="653.39105"
               y="1088.5574">TM</text>
               </g>
          </g>
        </g>
      </g>
    </g>
    <text
       id="text996-0"
       y="240.58926"
       x="417.28607"
       style="font-weight:800;font-size:80px;font-family:Arial-Black, 'Arial Black';fill:#ffffff;stroke-width:1.06528783">TURN</text>
    <path
       inkscape:connector-curvature="0"
       style="fill:#020202;fill-opacity:1;fill-rule:nonzero;stroke:none"
       d="M 274.02465,356.21704 H 803.14074 V 885.33309 H 274.02465 Z"
       id="path858" />
    <text
       y="149.75992"
       x="487.31073"
       id="text8-5"
       style="font-weight:800;font-size:24.34896851px;font-family:Arial-Black, 'Arial Black';fill:#cccccc;stroke-width:0.41538101">Pack: @(pack_id)</text>
    <text
       id="text843"
       y="1107.9752"
       x="361.87097"
       style="font-weight:800;font-size:12.79926109px;font-family:Arial-Black, 'Arial Black';fill:#e6e6e6;stroke-width:0.21834888">&#169; @(year) Ubiquity Robotics, Inc. All Rights Reserved</text>
  </g>

   <text
      style="font-weight:800;font-size:99.13722229px;font-family:Arial-Black, 'Arial Black';fill:#ffffff;stroke-width:0.21713541"
      id="text8"
      text-anchor="middle"
      x="50%"
      y="1070">@(num)</text>

                                            <g transform="translate(200,282) scale(1) translate(0, 0) scale(1) ">
						<rect x="@( (paper_width - fid_len)/2)mm" y="@( (paper_height - fid_len)/2)mm" width="@(fid_len)mm" height="4.0mm" style="stroke:none; fill:black"/>
						<rect x="@( (paper_width - fid_len)/2)mm" y="@( (paper_height + fid_len)/2 - 4)mm" width="@(fid_len)mm" height="4.0mm" style="stroke:none; fill:black"/>
						<rect x="@( (paper_width - fid_len)/2)mm" y="@( (paper_height - fid_len)/2)mm" width="4.0mm" height="@(fid_len)mm" style="stroke:none; fill:black"/>
						<rect x="@( (paper_width + fid_len)/2 - 4)mm" y="@( (paper_height - fid_len)/2)mm" width="4.0mm" height="@(fid_len)mm" style="stroke:none; fill:black"/>
						<image xmlns:xlink="http://www.w3.org/1999/xlink" x="@( (paper_width - fid_len)/2)mm" y="@( (paper_height - fid_len)/2)mm" width="@(fid_len)mm" height="@(fid_len)mm" xlink:href="@(marker_path)" />
					</g>
   <g transform = "translate(2,33) rotate(45 60 60)">
       <rect id = "myRect"
         x = "480"
         y = "0"
         height = "190"
         width = "65"
         stroke = "none"
         fill = "black" />
   </g>
				</svg>""", {"id": "%04d" % upper, "i": upper, "num": lower, "pack_id": pack_id, "paper_width": paper_size[0], "paper_height": paper_size[1], "fid_len": fid_len, "year": year, "marker_path": marker_path})

    elif tag == "BIDIR":  # Purple
        return em.expand("""<?xml version="1.0" encoding="UTF-8" standalone="no"?>
<svg
   xmlns:dc="http://purl.org/dc/elements/1.1/"
   xmlns:cc="http://creativecommons.org/ns#"
   xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#"
   xmlns:svg="http://www.w3.org/2000/svg"
   xmlns="http://www.w3.org/2000/svg"
   xmlns:sodipodi="http://sodipodi.sourceforge.net/DTD/sodipodi-0.dtd"
   xmlns:inkscape="http://www.inkscape.org/namespaces/inkscape"
   viewBox="0 0 1077.1654 1266.1414"
   version="1.1"
   id="svg28"
   sodipodi:docname="bidirectional.svg"
   width="28.5cm"
   height="33.5cm"
   inkscape:version="0.92.4 (5da689c313, 2019-01-14)">
  <metadata
     id="metadata34">
    <rdf:RDF>
      <cc:Work
         rdf:about="">
        <dc:format>image/svg+xml</dc:format>
        <dc:type
           rdf:resource="http://purl.org/dc/dcmitype/StillImage" />
        <dc:title>fiducial-markers</dc:title>
      </cc:Work>
    </rdf:RDF>
  </metadata>
  <defs
     id="defs32">
    <pattern
       y="0"
       x="0"
       height="6"
       width="6"
       patternUnits="userSpaceOnUse"
       id="EMFhbasepattern" />
  </defs>
  <sodipodi:namedview
     pagecolor="#ffffff"
     bordercolor="#666666"
     borderopacity="1"
     objecttolerance="10"
     gridtolerance="10"
     guidetolerance="10"
     inkscape:pageopacity="0"
     inkscape:pageshadow="2"
     inkscape:window-width="1920"
     inkscape:window-height="996"
     id="namedview30"
     showgrid="false"
     inkscape:zoom="0.5"
     inkscape:cx="653.91019"
     inkscape:cy="660.92449"
     inkscape:window-x="-8"
     inkscape:window-y="-8"
     inkscape:window-maximized="1"
     inkscape:current-layer="svg28"
     units="cm" />
  <title
     id="title2">fiducial-markers</title>
  <g
     id="g913">
    <text
       y="251.63947"
       x="210.81664"
       id="text8-4"
       style="font-weight:800;font-size:124.62126923px;font-family:Arial-Black, 'Arial Black';fill:#ffffff;stroke-width:0.27295187" />
    <path
       sodipodi:nodetypes="ccccccc"
       inkscape:connector-curvature="0"
       id="polygon4"
       d="M 179.52982,263.80704 V 1018.508 L 538.58268,1258.1547 897.63554,1018.508 V 263.80704 L 538.58269,7.9870361 Z"
       style="fill:#8462b6;fill-opacity:1;stroke-width:0.27037317" />
    <path
       inkscape:connector-curvature="0"
       style="fill:#000000;fill-opacity:1;fill-rule:nonzero;stroke:none"
       d="M 274.02465,356.21704 H 803.14074 V 885.33309 H 274.02465 Z"
       id="path858" />
    <text
       id="text996-0"
       y="232.01611"
       x="311.16406"
       style="font-weight:800;font-size:55px;font-family:Arial-Black, 'Arial Black';fill:#ffffff;stroke-width:0.73726732">BIDIRECTIONAL</text>
    <path
       sodipodi:nodetypes="cccc"
       inkscape:connector-curvature="0"
       id="polygon12"
       d="M 629.94292,113.03704 538.58431,46.317036 447.22239,113.03704 Z"
       style="fill:#ffffff;stroke-width:0.2904962" />
    <text
       y="149.75992"
       x="487.31073"
       id="text8-5"
       style="font-weight:800;font-size:24.34896851px;font-family:Arial-Black, 'Arial Black';fill:#cccccc;stroke-width:0.41538101">Pack: @(pack_id)</text>
    <g
       transform="translate(-209.54712,-106.72292)"
       id="g1061">
      <g
         transform="translate(-38.628653)"
         id="g930">
        <g
           transform="translate(2.8075834)"
           id="g846">
          <g
             transform="translate(-40)"
             id="g851">
            <path
               inkscape:connector-curvature="0"
               id="path10"
               style="fill:#ffffff;stroke-width:0.18099914"
               d="m 611.69999,1085.6544 v 0.2805 c 5.95847,1.3249 12.10882,4.2589 15.60754,8.5161 v 32.2649 c -5.10781,5.1095 -12.48894,7.8535 -18.06731,8.7042 3.87881,2.2715 7.09514,4.7312 9.55491,7.4752 2.0797,2.2716 4.82364,5.9622 8.13412,11.07 -0.94661,8.2318 -3.31048,15.1387 -10.02734,18.5451 l -11.44461,-17.1243 c -4.63537,-6.9087 -7.75758,-11.2599 -9.46079,-13.0572 -1.70321,-1.7974 -3.49871,-3.0282 -5.39198,-3.6907 -1.79732,-0.6624 -4.63359,-0.9466 -8.41828,-1.0407 v -14.194 h 9.08073 q 12.91249,0 16.17592,-1.1348 c 2.17199,-0.7565 3.7847,-1.991 5.01364,-3.7847 1.22899,-1.7937 1.79733,-3.9729 1.79733,-6.7168 0,-3.0282 -0.85067,-5.3938 -2.45977,-7.1912 -1.60727,-1.8932 -3.78287,-3.0281 -6.71508,-3.5023 -1.419,-0.181 -5.86619,-0.3783 -13.14955,-0.3783 h -9.74322 v -15.041 h -23.1679 v 53.1721 c 0,12.869 0.75661,22.5199 2.26976,29.1409 q 1.562,6.6716 6.52681,12.7731 c 3.31048,3.982 7.75763,7.287 13.33783,9.8409 5.67614,2.4597 14.00032,3.7847 24.97788,3.7847 9.08256,0 16.46008,-1.1349 22.04028,-3.4064 5.58203,-2.3657 10.12329,-5.488 13.43377,-9.3667 3.40459,-3.8788 5.6743,-8.7043 6.99924,-14.3822 1.23077,-5.6779 1.89143,-15.423 1.89143,-29.1409 v -52.4155 z" />
            <text
               y="1088.5574"
               x="653.39105"
               id="text8-5-4"
               style="font-weight:800;font-size:13.09123993px;font-family:Arial-Black, 'Arial Black';fill:#ffffff;stroke-width:0.2233299">TM</text>
          </g>
        </g>
      </g>
    </g>
    <text
       id="text843"
       y="1107.9752"
       x="361.87097"
       style="font-weight:800;font-size:12.79926109px;font-family:Arial-Black, 'Arial Black';fill:#e6e6e6;stroke-width:0.21834888">&#169; @(year) Ubiquity Robotics, Inc. All Rights Reserved</text>
    <path
       sodipodi:nodetypes="cccc"
       inkscape:connector-curvature="0"
       id="polygon12-24"
       d="m 448.6366,1159.037 91.35861,66.72 91.36192,-66.72 z"
       style="fill:#ffffff;stroke-width:0.2904962" />
  </g>

     <text
      style="font-weight:800;font-size:99.13722229px;font-family:Arial-Black, 'Arial Black';fill:#ffffff;stroke-width:0.21713541"
      id="text8"
      text-anchor="middle"
      x="50%"
      y="1070">@(num)</text>
                                                    <g transform="translate(200,282) scale(1) translate(0, 0) scale(1) ">
        						<rect x="@( (paper_width - fid_len)/2)mm" y="@( (paper_height - fid_len)/2)mm" width="@(fid_len)mm" height="4.0mm" style="stroke:none; fill:black"/>
                					<rect x="@( (paper_width - fid_len)/2)mm" y="@( (paper_height + fid_len)/2 - 4)mm" width="@(fid_len)mm" height="4.0mm" style="stroke:none; fill:black"/>
               						<rect x="@( (paper_width - fid_len)/2)mm" y="@( (paper_height - fid_len)/2)mm" width="4.0mm" height="@(fid_len)mm" style="stroke:none; fill:black"/>
               			 			<rect x="@( (paper_width + fid_len)/2 - 4)mm" y="@( (paper_height - fid_len)/2)mm" width="4.0mm" height="@(fid_len)mm" style="stroke:none; fill:black"/>
							<image xmlns:xlink="http://www.w3.org/1999/xlink" x="@( (paper_width - fid_len)/2)mm" y="@( (paper_height - fid_len)/2)mm" width="@(fid_len)mm" height="@(fid_len)mm" xlink:href="@(marker_path)" />
					</g>
   <g transform = "translate(2,33) rotate(45 60 60)">
       <rect id = "myRect"
         x = "480"
         y = "0"
         height = "190"
         width = "65"
         stroke = "none"
         fill = "black" />
   </g>
				</svg>""", {"id": "%04d" % upper, "i": upper, "num": lower, "pack_id": pack_id, "paper_width": paper_size[0], "paper_height": paper_size[1], "fid_len": fid_len, "year": year, "marker_path": marker_path})
    else:
        print("WARNING: Cannot return marker with ID %d." % upper)


def genFiducialwithBorder(upper, lower, tag, pack_id, marker_path, fid_len=140.0, year=2021):
    if tag == "GO":  # Green
        return em.expand("""<?xml version="1.0" encoding="UTF-8" standalone="no"?>
                                    <svg
   xmlns:dc="http://purl.org/dc/elements/1.1/"
   xmlns:cc="http://creativecommons.org/ns#"
   xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#"
   xmlns:svg="http://www.w3.org/2000/svg"
   xmlns="http://www.w3.org/2000/svg"
   xmlns:sodipodi="http://sodipodi.sourceforge.net/DTD/sodipodi-0.dtd"
   xmlns:inkscape="http://www.inkscape.org/namespaces/inkscape"
   viewBox="0 0 1077.1653 1266.1417"
   version="1.1"
   id="svg28"
   sodipodi:docname="go.svg"
   inkscape:version="0.92.4 (5da689c313, 2019-01-14)"
   width="28.5cm"
   height="33.5cm">
  <metadata
     id="metadata34">
    <rdf:RDF>
      <cc:Work
         rdf:about="">
        <dc:format>image/svg+xml</dc:format>
        <dc:type
           rdf:resource="http://purl.org/dc/dcmitype/StillImage" />
        <dc:title>fiducial-markers</dc:title>
      </cc:Work>
    </rdf:RDF>
  </metadata>
  <defs
     id="defs32" />
  <sodipodi:namedview
     pagecolor="#ffffff"
     bordercolor="#666666"
     borderopacity="1"
     objecttolerance="10"
     gridtolerance="10"
     guidetolerance="10"
     inkscape:pageopacity="0"
     inkscape:pageshadow="2"
     inkscape:window-width="1920"
     inkscape:window-height="996"
     id="namedview30"
     showgrid="false"
     inkscape:zoom="0.64"
     inkscape:cx="699.83079"
     inkscape:cy="788.03822"
     inkscape:window-x="-8"
     inkscape:window-y="-8"
     inkscape:window-maximized="1"
     inkscape:current-layer="g838"
     units="cm" />
  <title
     id="title2">fiducial-markers</title>
  <g
     id="g838"
     transform="translate(0,261842.8)">
    <g
       id="g846"
       transform="translate(-198.42516,363.604)">
      <path
         id="polygon4"
         transform="translate(0,-262353.04)"
         d="M 737.00781,154.63086 377.95508,410.44922 v 861.11128 h 179.52539 172.74805 v -6.4277 h 13.55859 v 6.4277 H 916.53516 1096.0605 V 410.44922 Z"
         style="fill:#57c15e;fill-opacity:1;stroke-width:0.27037317"
         inkscape:connector-curvature="0" />
      <path
         sodipodi:nodetypes="cccc"
         inkscape:connector-curvature="0"
         id="polygon12"
         d="m 828.36808,-262093.36 -91.35861,-66.72 -91.36192,66.72 z"
         style="fill:#ffffff;stroke-width:0.2904962" />
      <g
         transform="translate(-11.12196,-262313.12)"
         id="g1061">
        <g
           transform="translate(-38.628653)"
           id="g930">
          <g
             transform="translate(41.339389)"
             id="g843">
            <text
               style="font-weight:800;font-size:99.13722229px;font-family:Arial-Black, 'Arial Black';fill:#ffffff;stroke-width:0.21713541"
               id="text8"
               x="671.1004"
               y="1169.3763">@(num)</text>
            <path
               d="m 611.69999,1085.6544 v 0.2805 c 5.95847,1.3249 12.10882,4.2589 15.60754,8.5161 v 32.2649 c -5.10781,5.1095 -12.48894,7.8535 -18.06731,8.7042 3.87881,2.2715 7.09514,4.7312 9.55491,7.4752 2.0797,2.2716 4.82364,5.9622 8.13412,11.07 -0.94661,8.2318 -3.31048,15.1387 -10.02734,18.5451 l -11.44461,-17.1243 c -4.63537,-6.9087 -7.75758,-11.2599 -9.46079,-13.0572 -1.70321,-1.7974 -3.49871,-3.0282 -5.39198,-3.6907 -1.79732,-0.6624 -4.63359,-0.9466 -8.41828,-1.0407 v -14.194 h 9.08073 q 12.91249,0 16.17592,-1.1348 c 2.17199,-0.7565 3.7847,-1.991 5.01364,-3.7847 1.22899,-1.7937 1.79733,-3.9729 1.79733,-6.7168 0,-3.0282 -0.85067,-5.3938 -2.45977,-7.1912 -1.60727,-1.8932 -3.78287,-3.0281 -6.71508,-3.5023 -1.419,-0.181 -5.86619,-0.3783 -13.14955,-0.3783 h -9.74322 v -15.041 h -23.1679 v 53.1721 c 0,12.869 0.75661,22.5199 2.26976,29.1409 q 1.562,6.6716 6.52681,12.7731 c 3.31048,3.982 7.75763,7.287 13.33783,9.8409 5.67614,2.4597 14.00032,3.7847 24.97788,3.7847 9.08256,0 16.46008,-1.1349 22.04028,-3.4064 5.58203,-2.3657 10.12329,-5.488 13.43377,-9.3667 3.40459,-3.8788 5.6743,-8.7043 6.99924,-14.3822 1.23077,-5.6779 1.89143,-15.423 1.89143,-29.1409 v -52.4155 z"
               style="fill:#ffffff;stroke-width:0.18099914"
               id="path10"
               inkscape:connector-curvature="0" />
            <text
               style="font-weight:800;font-size:13.09123993px;font-family:Arial-Black, 'Arial Black';fill:#ffffff;stroke-width:0.2233299"
               id="text8-5-4"
               x="653.39105"
               y="1088.5574">TM</text>
          </g>
        </g>
      </g>
      <text
         id="text996-0"
         y="-261989.62"
         x="684.95978"
         style="font-weight:800;font-size:62.44546127px;font-family:Arial-Black, 'Arial Black';fill:#ffffff;stroke-width:1.06528783">GO</text>
      <g
         id="g989"
         transform="translate(3.5546875e-6,-262353.04)">
        <rect
           ry="7.0710678"
           rx="7.0710678"
           x="395.27161"
           y="426.82996"
           width="683.47247"
           height="681.17603"
           style="fill:#ffffff;stroke-width:0.33476147"
           id="rect8" />
        <path
           id="path858"
           d="M 472.44981,502.85995 H 1001.5659 V 1031.976 H 472.44981 Z"
           style="fill:#020202;fill-opacity:1;fill-rule:nonzero;stroke:none"
           inkscape:connector-curvature="0" />
      </g>
      <text
         id="text843"
         y="-261098.42"
         x="560.29614"
         style="font-weight:800;font-size:12.79926109px;font-family:Arial-Black, 'Arial Black';fill:#e6e6e6;stroke-width:0.21834888">&#169; @(year) Ubiquity Robotics, Inc. All Rights Reserved</text>
      <text
         y="-262056.64"
         x="685.7359"
         id="text8-5"
         style="font-weight:800;font-size:24.34896851px;font-family:Arial-Black, 'Arial Black';fill:#cccccc;stroke-width:0.41538101">Pack: @(pack_id)</text>
      <text
         id="text996"
         y="-261949.67"
         x="666.43304"
         style="font-weight:800;font-size:17.55582809px;font-family:Arial-Black, 'Arial Black';fill:#cccccc;stroke-width:0.29949352">Fiducial #@(id)</text>
    </g>
  </g>
                                        <g transform="translate(200,282) scale(1) translate(0, 0) scale(1) ">
                                                <rect x="@( (paper_width - fid_len)/2)mm" y="@( (paper_height - fid_len)/2)mm" width="@(fid_len)mm" height="4.0mm" style="stroke:none; fill:black"/>
                                                <rect x="@( (paper_width - fid_len)/2)mm" y="@( (paper_height + fid_len)/2 - 4)mm" width="@(fid_len)mm" height="4.0mm" style="stroke:none; fill:black"/>
                                                <rect x="@( (paper_width - fid_len)/2)mm" y="@( (paper_height - fid_len)/2)mm" width="4.0mm" height="@(fid_len)mm" style="stroke:none; fill:black"/>
                                                <rect x="@( (paper_width + fid_len)/2 - 4)mm" y="@( (paper_height - fid_len)/2)mm" width="4.0mm" height="@(fid_len)mm" style="stroke:none; fill:black"/>
                                                <image xmlns:xlink="http://www.w3.org/1999/xlink" x="@( (paper_width - fid_len)/2)mm" y="@( (paper_height - fid_len)/2)mm" width="@(fid_len)mm" height="@(fid_len)mm" xlink:href="@(marker_path)" />
                                        </g>
                                        </svg>""", {"id": "%04d" % upper, "i": upper, "num": lower, "pack_id": pack_id, "paper_width": paper_size[0], "paper_height": paper_size[1], "fid_len": fid_len, "year": year, "marker_path": marker_path})

    elif tag == "STOP":  # Red
        return em.expand("""<?xml version="1.0" encoding="UTF-8" standalone="no"?>
                                    <svg
   xmlns:dc="http://purl.org/dc/elements/1.1/"
   xmlns:cc="http://creativecommons.org/ns#"
   xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#"
   xmlns:svg="http://www.w3.org/2000/svg"
   xmlns="http://www.w3.org/2000/svg"
   xmlns:sodipodi="http://sodipodi.sourceforge.net/DTD/sodipodi-0.dtd"
   xmlns:inkscape="http://www.inkscape.org/namespaces/inkscape"
   viewBox="0 0 1077.1648 1266.1415"
   version="1.1"
   id="svg28"
   sodipodi:docname="stop.svg"
   inkscape:version="0.92.4 (5da689c313, 2019-01-14)"
   width="28.5cm"
   height="33.5cm">
  <metadata
     id="metadata34">
    <rdf:RDF>
      <cc:Work
         rdf:about="">
        <dc:format>image/svg+xml</dc:format>
        <dc:type
           rdf:resource="http://purl.org/dc/dcmitype/StillImage" />
        <dc:title>fiducial-markers</dc:title>
      </cc:Work>
    </rdf:RDF>
  </metadata>
  <defs
     id="defs32">
    <pattern
       y="0"
       x="0"
       height="6"
       width="6"
       patternUnits="userSpaceOnUse"
       id="EMFhbasepattern" />
  </defs>
  <sodipodi:namedview
     pagecolor="#ffffff"
     bordercolor="#666666"
     borderopacity="1"
     objecttolerance="10"
     gridtolerance="10"
     guidetolerance="10"
     inkscape:pageopacity="0"
     inkscape:pageshadow="2"
     inkscape:window-width="1920"
     inkscape:window-height="996"
     id="namedview30"
     showgrid="false"
     inkscape:zoom="0.5"
     inkscape:cx="705.74685"
     inkscape:cy="727.82485"
     inkscape:window-x="-8"
     inkscape:window-y="-8"
     inkscape:window-maximized="1"
     inkscape:current-layer="svg28"
     units="cm" />
  <title
     id="title2">fiducial-markers</title>
  <g
     id="g852"
     transform="translate(-198.42515,-146.64058)">
    <path
       id="path4"
       d="M 519.25586,266.3457 211.31055,567.22852 v 425.51171 l 307.94531,300.89067 210.97266,-0.01 v -11.9297 h 13.55859 v 11.9297 l 210.96875,-0.01 307.94924,-300.89058 v -425.5 L 954.76172,266.3457 H 743.78711 v 10.3711 h -13.55859 v -10.3711 z"
       style="fill:#cb4d4d;stroke-width:0.26510242"
       inkscape:connector-curvature="0" />
    <g
       id="g989"
       transform="translate(3.5546875e-6,-3.2987814e-6)">
      <rect
         ry="7.0710678"
         rx="7.0710678"
         x="395.27161"
         y="426.82996"
         width="683.47247"
         height="681.17603"
         style="fill:#ffffff;stroke-width:0.33476147"
         id="rect8" />
      <path
         id="path858"
         d="M 472.44981,502.85995 H 1001.5659 V 1031.976 H 472.44981 Z"
         style="fill:#020202;fill-opacity:1;fill-rule:nonzero;stroke:none"
         inkscape:connector-curvature="0" />
    </g>
    <text
       id="text996-0"
       y="369.92352"
       x="644.57458"
       style="font-weight:800;font-size:62.44546127px;font-family:Arial-Black, 'Arial Black';fill:#ffffff;stroke-width:1.06528783">STOP</text>
    <text
       y="305.62317"
       x="685.7359"
       id="text8-5"
       style="font-weight:800;font-size:24.34896851px;font-family:Arial-Black, 'Arial Black';fill:#cccccc;stroke-width:0.41538101">Pack: @(pack_id)</text>
    <path
       inkscape:connector-curvature="0"
       id="rect1123"
       d="m 528.48438,279.13281 v 8.94439 h 92.50585 v -8.94439 z m 324.54101,0 v 8.94439 h 97.32617 v -8.94439 z"
       style="fill:#ffffff;fill-opacity:1;stroke:none;stroke-width:0.72918367;stroke-linecap:round;stroke-linejoin:round;stroke-miterlimit:4;stroke-dasharray:none;stroke-opacity:1" />
    <g
       transform="translate(0.46542268,18.932422)"
       id="g1242">
      <polygon
         points="2180.42,916.32 2180.42,659.13 2334.51,659.13 2059.24,397.68 1783.98,659.13 1938.07,659.13 1938.07,916.32 "
         style="fill:#ffffff"
         id="polygon6"
         transform="matrix(0.21393362,0,0,0.21393362,-138.9824,620.50149)" />
      <polygon
         points="1938.07,916.32 2180.42,916.32 2180.42,659.13 2334.51,659.13 2059.24,397.68 1783.98,659.13 1938.07,659.13 "
         style="fill:#ffffff"
         id="polygon6-6"
         transform="matrix(0.21393362,0,0,0.21393362,730.98373,620.50149)" />
    </g>
    <g
       transform="translate(-11.121957,39.919995)"
       id="g1061">
      <g
         transform="translate(-38.628653)"
         id="g930">
        <g
           transform="translate(5.5183533)"
           id="g851">
          <g
             transform="translate(35.821068)"
             id="g858">
            <text
               y="1169.3763"
               x="671.1004"
               id="text8"
               style="font-weight:800;font-size:99.13722229px;font-family:Arial-Black, 'Arial Black';fill:#ffffff;stroke-width:0.21713541">@(num)</text>
            <path
               inkscape:connector-curvature="0"
               id="path10"
               style="fill:#ffffff;stroke-width:0.18099914"
               d="m 611.69999,1085.6544 v 0.2805 c 5.95847,1.3249 12.10882,4.2589 15.60754,8.5161 v 32.2649 c -5.10781,5.1095 -12.48894,7.8535 -18.06731,8.7042 3.87881,2.2715 7.09514,4.7312 9.55491,7.4752 2.0797,2.2716 4.82364,5.9622 8.13412,11.07 -0.94661,8.2318 -3.31048,15.1387 -10.02734,18.5451 l -11.44461,-17.1243 c -4.63537,-6.9087 -7.75758,-11.2599 -9.46079,-13.0572 -1.70321,-1.7974 -3.49871,-3.0282 -5.39198,-3.6907 -1.79732,-0.6624 -4.63359,-0.9466 -8.41828,-1.0407 v -14.194 h 9.08073 q 12.91249,0 16.17592,-1.1348 c 2.17199,-0.7565 3.7847,-1.991 5.01364,-3.7847 1.22899,-1.7937 1.79733,-3.9729 1.79733,-6.7168 0,-3.0282 -0.85067,-5.3938 -2.45977,-7.1912 -1.60727,-1.8932 -3.78287,-3.0281 -6.71508,-3.5023 -1.419,-0.181 -5.86619,-0.3783 -13.14955,-0.3783 h -9.74322 v -15.041 h -23.1679 v 53.1721 c 0,12.869 0.75661,22.5199 2.26976,29.1409 q 1.562,6.6716 6.52681,12.7731 c 3.31048,3.982 7.75763,7.287 13.33783,9.8409 5.67614,2.4597 14.00032,3.7847 24.97788,3.7847 9.08256,0 16.46008,-1.1349 22.04028,-3.4064 5.58203,-2.3657 10.12329,-5.488 13.43377,-9.3667 3.40459,-3.8788 5.6743,-8.7043 6.99924,-14.3822 1.23077,-5.6779 1.89143,-15.423 1.89143,-29.1409 v -52.4155 z" />
            <text
               y="1088.5574"
               x="653.39105"
               id="text8-5-4"
               style="font-weight:800;font-size:13.09123993px;font-family:Arial-Black, 'Arial Black';fill:#ffffff;stroke-width:0.2233299">TM</text>
          </g>
        </g>
      </g>
    </g>
    <text
       id="text996"
       y="403.37158"
       x="666.43304"
       style="font-weight:800;font-size:17.55582809px;font-family:Arial-Black, 'Arial Black';fill:#cccccc;stroke-width:0.29949352">Fiducial #@(id)</text>
    <text
       id="text843"
       y="1254.6182"
       x="560.29614"
       style="font-weight:800;font-size:12.79926109px;font-family:Arial-Black, 'Arial Black';fill:#e6e6e6;stroke-width:0.21834888">&#169; @(year) Ubiquity Robotics, Inc. All Rights Reserved</text>
  </g>
                                            <g transform="translate(200,282) scale(1) translate(0, 0) scale(1) ">
                                                    <rect x="@( (paper_width - fid_len)/2)mm" y="@( (paper_height - fid_len)/2)mm" width="@(fid_len)mm" height="4.0mm" style="stroke:none; fill:black"/>
               					<rect x="@( (paper_width - fid_len)/2)mm" y="@( (paper_height + fid_len)/2 - 4)mm" width="@(fid_len)mm" height="4.0mm" style="stroke:none; fill:black"/>
                				<rect x="@( (paper_width - fid_len)/2)mm" y="@( (paper_height - fid_len)/2)mm" width="4.0mm" height="@(fid_len)mm" style="stroke:none; fill:black"/>
                				<rect x="@( (paper_width + fid_len)/2 - 4)mm" y="@( (paper_height - fid_len)/2)mm" width="4.0mm" height="@(fid_len)mm" style="stroke:none; fill:black"/>
                				<image xmlns:xlink="http://www.w3.org/1999/xlink" x="@( (paper_width - fid_len)/2)mm" y="@( (paper_height - fid_len)/2)mm" width="@(fid_len)mm" height="@(fid_len)mm" xlink:href="@(marker_path)" />
					</g>
				</svg>""", {"id": "%04d" % upper, "i": upper, "num": lower, "pack_id": pack_id, "paper_width": paper_size[0], "paper_height": paper_size[1], "fid_len": fid_len, "year": year, "marker_path": marker_path})

    elif tag == "TURN":  # Blue
        return em.expand("""<?xml version="1.0" encoding="UTF-8" standalone="no"?>
                                    <svg
   xmlns:dc="http://purl.org/dc/elements/1.1/"
   xmlns:cc="http://creativecommons.org/ns#"
   xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#"
   xmlns:svg="http://www.w3.org/2000/svg"
   xmlns="http://www.w3.org/2000/svg"
   xmlns:sodipodi="http://sodipodi.sourceforge.net/DTD/sodipodi-0.dtd"
   xmlns:inkscape="http://www.inkscape.org/namespaces/inkscape"
   viewBox="0 0 1077.1653 1266.1417"
   version="1.1"
   id="svg38"
   sodipodi:docname="turn.svg"
   inkscape:version="0.92.4 (5da689c313, 2019-01-14)"
   width="28.5cm"
   height="33.5cm">
  <metadata
     id="metadata44">
    <rdf:RDF>
      <cc:Work
         rdf:about="">
        <dc:format>image/svg+xml</dc:format>
        <dc:type
           rdf:resource="http://purl.org/dc/dcmitype/StillImage" />
        <dc:title>fiducial-markers</dc:title>
      </cc:Work>
    </rdf:RDF>
  </metadata>
  <defs
     id="defs42">
    <pattern
       y="0"
       x="0"
       height="6"
       width="6"
       patternUnits="userSpaceOnUse"
       id="EMFhbasepattern" />
  </defs>
  <sodipodi:namedview
     pagecolor="#ffffff"
     bordercolor="#666666"
     borderopacity="1"
     objecttolerance="10"
     gridtolerance="10"
     guidetolerance="10"
     inkscape:pageopacity="0"
     inkscape:pageshadow="2"
     inkscape:window-width="1920"
     inkscape:window-height="996"
     id="namedview40"
     showgrid="false"
     inkscape:zoom="0.35355339"
     inkscape:cx="699.49233"
     inkscape:cy="875.01009"
     inkscape:window-x="-8"
     inkscape:window-y="-8"
     inkscape:window-maximized="1"
     inkscape:current-layer="svg38"
     units="cm" />
  <title
     id="title2">fiducial-markers</title>
  <g
     id="g845"
     transform="translate(-198.42516,-146.64291)">
    <path
       sodipodi:nodetypes="cccccccccccccccc"
       inkscape:connector-curvature="0"
       id="polygon4"
       d="m 737.00781,158.43359 -89.76318,63.95557 -89.76319,63.95557 -179.52636,127.91113 v 861.11134 h 179.52539 172.74805 v -6.7847 h 13.55859 v 6.7847 H 916.53516 1096.0605 V 414.25586 L 916.53416,286.34473 826.77098,222.38916 Z"
       style="fill:#5391c5;stroke-width:0.27037317" />
    <path
       sodipodi:nodetypes="cccc"
       inkscape:connector-curvature="0"
       id="polygon12"
       d="M 828.36808,263.48599 737.00947,196.7647 645.64755,263.48599 Z"
       style="fill:#ffffff;stroke-width:0.2904962" />
    <g
       transform="translate(24.699109,43.720771)"
       id="g1061">
      <g
         transform="translate(2e-6,-3.8007758)"
         id="g1061-0">
        <g
           transform="translate(-38.628653)"
           id="g930">
          <g
             transform="translate(5.5183183)"
             id="g857">
            <text
               style="font-weight:800;font-size:99.13722229px;font-family:Arial-Black, 'Arial Black';fill:#ffffff;stroke-width:0.21713541"
               id="text8"
               x="671.1004"
               y="1169.3763">@(num)</text>
            <path
               d="m 611.69999,1085.6544 v 0.2805 c 5.95847,1.3249 12.10882,4.2589 15.60754,8.5161 v 32.2649 c -5.10781,5.1095 -12.48894,7.8535 -18.06731,8.7042 3.87881,2.2715 7.09514,4.7312 9.55491,7.4752 2.0797,2.2716 4.82364,5.9622 8.13412,11.07 -0.94661,8.2318 -3.31048,15.1387 -10.02734,18.5451 l -11.44461,-17.1243 c -4.63537,-6.9087 -7.75758,-11.2599 -9.46079,-13.0572 -1.70321,-1.7974 -3.49871,-3.0282 -5.39198,-3.6907 -1.79732,-0.6624 -4.63359,-0.9466 -8.41828,-1.0407 v -14.194 h 9.08073 q 12.91249,0 16.17592,-1.1348 c 2.17199,-0.7565 3.7847,-1.991 5.01364,-3.7847 1.22899,-1.7937 1.79733,-3.9729 1.79733,-6.7168 0,-3.0282 -0.85067,-5.3938 -2.45977,-7.1912 -1.60727,-1.8932 -3.78287,-3.0281 -6.71508,-3.5023 -1.419,-0.181 -5.86619,-0.3783 -13.14955,-0.3783 h -9.74322 v -15.041 h -23.1679 v 53.1721 c 0,12.869 0.75661,22.5199 2.26976,29.1409 q 1.562,6.6716 6.52681,12.7731 c 3.31048,3.982 7.75763,7.287 13.33783,9.8409 5.67614,2.4597 14.00032,3.7847 24.97788,3.7847 9.08256,0 16.46008,-1.1349 22.04028,-3.4064 5.58203,-2.3657 10.12329,-5.488 13.43377,-9.3667 3.40459,-3.8788 5.6743,-8.7043 6.99924,-14.3822 1.23077,-5.6779 1.89143,-15.423 1.89143,-29.1409 v -52.4155 z"
               style="fill:#ffffff;stroke-width:0.18099914"
               id="path10"
               inkscape:connector-curvature="0" />
            <text
               style="font-weight:800;font-size:13.09123993px;font-family:Arial-Black, 'Arial Black';fill:#ffffff;stroke-width:0.2233299"
               id="text8-5-4"
               x="653.39105"
               y="1088.5574">TM</text>
          </g>
        </g>
      </g>
    </g>
    <text
       id="text996-0"
       y="367.23218"
       x="639.71124"
       style="font-weight:800;font-size:62.44546127px;font-family:Arial-Black, 'Arial Black';fill:#ffffff;stroke-width:1.06528783">TURN</text>
    <g
       id="g989"
       transform="translate(3.5546875e-6,-3.2987814e-6)">
      <rect
         ry="7.0710678"
         rx="7.0710678"
         x="395.27161"
         y="426.82996"
         width="683.47247"
         height="681.17603"
         style="fill:#ffffff;stroke-width:0.33476147"
         id="rect8" />
      <path
         id="path858"
         d="M 472.44981,502.85995 H 1001.5659 V 1031.976 H 472.44981 Z"
         style="fill:#020202;fill-opacity:1;fill-rule:nonzero;stroke:none"
         inkscape:connector-curvature="0" />
    </g>
    <text
       y="296.40283"
       x="685.7359"
       id="text8-5"
       style="font-weight:800;font-size:24.34896851px;font-family:Arial-Black, 'Arial Black';fill:#cccccc;stroke-width:0.41538101">Pack: @(pack_id)</text>
    <text
       id="text996"
       y="403.37158"
       x="666.43304"
       style="font-weight:800;font-size:17.55582809px;font-family:Arial-Black, 'Arial Black';fill:#cccccc;stroke-width:0.29949352">Fiducial #@(id)</text>
    <text
       id="text843"
       y="1254.6182"
       x="560.29614"
       style="font-weight:800;font-size:12.79926109px;font-family:Arial-Black, 'Arial Black';fill:#e6e6e6;stroke-width:0.21834888">&#169; @(year) Ubiquity Robotics, Inc. All Rights Reserved</text>
  </g>
                                            <g transform="translate(200,282) scale(1) translate(0, 0) scale(1) ">
						<rect x="@( (paper_width - fid_len)/2)mm" y="@( (paper_height - fid_len)/2)mm" width="@(fid_len)mm" height="4.0mm" style="stroke:none; fill:black"/>
						<rect x="@( (paper_width - fid_len)/2)mm" y="@( (paper_height + fid_len)/2 - 4)mm" width="@(fid_len)mm" height="4.0mm" style="stroke:none; fill:black"/>
						<rect x="@( (paper_width - fid_len)/2)mm" y="@( (paper_height - fid_len)/2)mm" width="4.0mm" height="@(fid_len)mm" style="stroke:none; fill:black"/>
						<rect x="@( (paper_width + fid_len)/2 - 4)mm" y="@( (paper_height - fid_len)/2)mm" width="4.0mm" height="@(fid_len)mm" style="stroke:none; fill:black"/>
						<image xmlns:xlink="http://www.w3.org/1999/xlink" x="@( (paper_width - fid_len)/2)mm" y="@( (paper_height - fid_len)/2)mm" width="@(fid_len)mm" height="@(fid_len)mm" xlink:href="@(marker_path)" />
					</g>
				</svg>""", {"id": "%04d" % upper, "i": upper, "num": lower, "pack_id": pack_id, "paper_width": paper_size[0], "paper_height": paper_size[1], "fid_len": fid_len, "year": year, "marker_path": marker_path})

    elif tag == "BIDIR":  # Purple
        return em.expand("""<?xml version="1.0" encoding="UTF-8" standalone="no"?>
                                    <svg
   xmlns:dc="http://purl.org/dc/elements/1.1/"
   xmlns:cc="http://creativecommons.org/ns#"
   xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#"
   xmlns:svg="http://www.w3.org/2000/svg"
   xmlns="http://www.w3.org/2000/svg"
   xmlns:sodipodi="http://sodipodi.sourceforge.net/DTD/sodipodi-0.dtd"
   xmlns:inkscape="http://www.inkscape.org/namespaces/inkscape"
   viewBox="0 0 1077.1654 1266.1414"
   version="1.1"
   id="svg28"
   sodipodi:docname="bidirectional.svg"
   width="28.5cm"
   height="33.5cm"
   inkscape:version="0.92.4 (5da689c313, 2019-01-14)">
  <metadata
     id="metadata34">
    <rdf:RDF>
      <cc:Work
         rdf:about="">
        <dc:format>image/svg+xml</dc:format>
        <dc:type
           rdf:resource="http://purl.org/dc/dcmitype/StillImage" />
        <dc:title>fiducial-markers</dc:title>
      </cc:Work>
    </rdf:RDF>
  </metadata>
  <defs
     id="defs32">
    <pattern
       y="0"
       x="0"
       height="6"
       width="6"
       patternUnits="userSpaceOnUse"
       id="EMFhbasepattern" />
  </defs>
  <sodipodi:namedview
     pagecolor="#ffffff"
     bordercolor="#666666"
     borderopacity="1"
     objecttolerance="10"
     gridtolerance="10"
     guidetolerance="10"
     inkscape:pageopacity="0"
     inkscape:pageshadow="2"
     inkscape:window-width="1920"
     inkscape:window-height="996"
     id="namedview30"
     showgrid="false"
     inkscape:zoom="0.5"
     inkscape:cx="693.06361"
     inkscape:cy="693.3777"
     inkscape:window-x="-8"
     inkscape:window-y="-8"
     inkscape:window-maximized="1"
     inkscape:current-layer="svg28"
     units="cm" />
  <title
     id="title2">fiducial-markers</title>
  <text
     style="font-weight:800;font-size:124.62126923px;font-family:Arial-Black, 'Arial Black';fill:#ffffff;stroke-width:0.27295187"
     id="text8-4"
     x="210.81664"
     y="251.63947" />
  <g
     id="g942">
    <path
       sodipodi:nodetypes="ccccccc"
       inkscape:connector-curvature="0"
       id="polygon4"
       d="M 179.52982,263.80704 V 1018.508 L 538.58268,1258.1547 897.63554,1018.508 V 263.80704 L 538.58269,7.9870361 Z"
       style="fill:#8462b6;fill-opacity:1;stroke-width:0.27037317" />
    <rect
       id="rect8"
       style="fill:#ffffff;stroke-width:0.33476147"
       height="681.17603"
       width="683.47247"
       y="280.18704"
       x="196.84645"
       rx="7.0710678"
       ry="7.0710678" />
    <path
       inkscape:connector-curvature="0"
       style="fill:#000000;fill-opacity:1;fill-rule:nonzero;stroke:none"
       d="M 274.02465,356.21704 H 803.14074 V 885.33309 H 274.02465 Z"
       id="path858" />
    <path
       sodipodi:nodetypes="cccc"
       inkscape:connector-curvature="0"
       id="polygon12-2"
       d="m 426.89607,1152.6336 111.68456,73.5119 111.68861,-73.5119 z"
       style="fill:#ffffff;stroke-width:0.3371391" />
    <text
       id="text996-0"
       y="212.01611"
       x="355.16406"
       style="font-weight:800;font-size:43.21743011px;font-family:Arial-Black, 'Arial Black';fill:#ffffff;stroke-width:0.73726732">BIDIRECTIONAL</text>
    <path
       sodipodi:nodetypes="cccc"
       inkscape:connector-curvature="0"
       id="polygon12"
       d="M 629.94292,113.03704 538.58431,46.317036 447.22239,113.03704 Z"
       style="fill:#ffffff;stroke-width:0.2904962" />
    <text
       y="149.75992"
       x="487.31073"
       id="text8-5"
       style="font-weight:800;font-size:24.34896851px;font-family:Arial-Black, 'Arial Black';fill:#cccccc;stroke-width:0.41538101">Pack: @(pack_id)</text>
    <text
       id="text996"
       y="256.7287"
       x="468.00787"
       style="font-weight:800;font-size:17.55582809px;font-family:Arial-Black, 'Arial Black';fill:#cccccc;stroke-width:0.29949352">Fiducial #@(id)</text>
    <g
       transform="translate(-209.54712,-106.72292)"
       id="g1061">
      <g
         transform="translate(-38.628653)"
         id="g930">
        <g
           transform="translate(2.8075834)"
           id="g846">
          <g
             transform="translate(38.531853)"
             id="g851">
            <text
               y="1169.3763"
               x="671.1004"
               id="text8"
               style="font-weight:800;font-size:99.13722229px;font-family:Arial-Black, 'Arial Black';fill:#ffffff;stroke-width:0.21713541">@(num)</text>
            <path
               inkscape:connector-curvature="0"
               id="path10"
               style="fill:#ffffff;stroke-width:0.18099914"
               d="m 611.69999,1085.6544 v 0.2805 c 5.95847,1.3249 12.10882,4.2589 15.60754,8.5161 v 32.2649 c -5.10781,5.1095 -12.48894,7.8535 -18.06731,8.7042 3.87881,2.2715 7.09514,4.7312 9.55491,7.4752 2.0797,2.2716 4.82364,5.9622 8.13412,11.07 -0.94661,8.2318 -3.31048,15.1387 -10.02734,18.5451 l -11.44461,-17.1243 c -4.63537,-6.9087 -7.75758,-11.2599 -9.46079,-13.0572 -1.70321,-1.7974 -3.49871,-3.0282 -5.39198,-3.6907 -1.79732,-0.6624 -4.63359,-0.9466 -8.41828,-1.0407 v -14.194 h 9.08073 q 12.91249,0 16.17592,-1.1348 c 2.17199,-0.7565 3.7847,-1.991 5.01364,-3.7847 1.22899,-1.7937 1.79733,-3.9729 1.79733,-6.7168 0,-3.0282 -0.85067,-5.3938 -2.45977,-7.1912 -1.60727,-1.8932 -3.78287,-3.0281 -6.71508,-3.5023 -1.419,-0.181 -5.86619,-0.3783 -13.14955,-0.3783 h -9.74322 v -15.041 h -23.1679 v 53.1721 c 0,12.869 0.75661,22.5199 2.26976,29.1409 q 1.562,6.6716 6.52681,12.7731 c 3.31048,3.982 7.75763,7.287 13.33783,9.8409 5.67614,2.4597 14.00032,3.7847 24.97788,3.7847 9.08256,0 16.46008,-1.1349 22.04028,-3.4064 5.58203,-2.3657 10.12329,-5.488 13.43377,-9.3667 3.40459,-3.8788 5.6743,-8.7043 6.99924,-14.3822 1.23077,-5.6779 1.89143,-15.423 1.89143,-29.1409 v -52.4155 z" />
            <text
               y="1088.5574"
               x="653.39105"
               id="text8-5-4"
               style="font-weight:800;font-size:13.09123993px;font-family:Arial-Black, 'Arial Black';fill:#ffffff;stroke-width:0.2233299">TM</text>
          </g>
        </g>
      </g>
    </g>
    <text
       id="text843"
       y="1107.9752"
       x="361.87097"
       style="font-weight:800;font-size:12.79926109px;font-family:Arial-Black, 'Arial Black';fill:#e6e6e6;stroke-width:0.21834888">&#169; @(year) Ubiquity Robotics, Inc. All Rights Reserved</text>
  </g>
                                                    <g transform="translate(200,282) scale(1) translate(0, 0) scale(1) ">
        						<rect x="@( (paper_width - fid_len)/2)mm" y="@( (paper_height - fid_len)/2)mm" width="@(fid_len)mm" height="4.0mm" style="stroke:none; fill:black"/>
                					<rect x="@( (paper_width - fid_len)/2)mm" y="@( (paper_height + fid_len)/2 - 4)mm" width="@(fid_len)mm" height="4.0mm" style="stroke:none; fill:black"/>
               						<rect x="@( (paper_width - fid_len)/2)mm" y="@( (paper_height - fid_len)/2)mm" width="4.0mm" height="@(fid_len)mm" style="stroke:none; fill:black"/>
               			 			<rect x="@( (paper_width + fid_len)/2 - 4)mm" y="@( (paper_height - fid_len)/2)mm" width="4.0mm" height="@(fid_len)mm" style="stroke:none; fill:black"/>
							<image xmlns:xlink="http://www.w3.org/1999/xlink" x="@( (paper_width - fid_len)/2)mm" y="@( (paper_height - fid_len)/2)mm" width="@(fid_len)mm" height="@(fid_len)mm" xlink:href="@(marker_path)" />
					</g>
				</svg>""", {"id": "%04d" % upper, "i": upper, "num": lower, "pack_id": pack_id, "paper_width": paper_size[0], "paper_height": paper_size[1], "fid_len": fid_len, "year": year, "marker_path": marker_path})
    else:
        print("WARNING: Cannot return marker with ID %d." % upper)


if __name__ == "__main__":
    import os
    marker_path ="/home/mabo/STag Markers/HD11/00010.png"
    print("Exists:", os.path.exists(marker_path))
    print("Is file:", os.path.isfile(marker_path))
    print("Path:", marker_path)
    marker_path2 = "file://" + quote(marker_path)

    svg = genFiducialwithoutBorder(10, 10, "GO", 11, marker_path, fid_len=184.5)
    with open("output.svg", "w") as f:
      f.write(svg)
