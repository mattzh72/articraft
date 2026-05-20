from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ClevisBracketGeometry,
    Cylinder,
    Material,
    MeshGeometry,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


SHAFT_RPY = {
    "x": (0.0, math.pi / 2.0, 0.0),
    "y": (-math.pi / 2.0, 0.0, 0.0),
    "z": (0.0, 0.0, 0.0),
}


def _cyl_on_axis(
    part,
    *,
    radius: float,
    length: float,
    axis: str,
    xyz: tuple[float, float, float],
    material: Material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=SHAFT_RPY[axis]),
        material=material,
        name=name,
    )


def _add_box(
    part,
    *,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: Material,
    name: str,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _ring_indices(
    geom: MeshGeometry,
    *,
    z: float,
    outer_profile: list[float],
    inner_radius: float,
) -> tuple[list[int], list[int]]:
    outer_ids: list[int] = []
    inner_ids: list[int] = []
    count = len(outer_profile)
    for i, radius in enumerate(outer_profile):
        angle = 2.0 * math.pi * i / count
        outer_ids.append(geom.add_vertex(radius * math.cos(angle), radius * math.sin(angle), z))
    for i in range(count):
        angle = 2.0 * math.pi * i / count
        inner_ids.append(geom.add_vertex(inner_radius * math.cos(angle), inner_radius * math.sin(angle), z))
    return outer_ids, inner_ids


def _tooth_profile(teeth: int, root_radius: float, outer_radius: float) -> list[float]:
    profile: list[float] = []
    for _ in range(teeth):
        profile.extend((root_radius, outer_radius, outer_radius, root_radius))
    return profile


def _toothed_ring_geometry(
    *,
    teeth: int,
    root_radius: float,
    outer_radius: float,
    inner_radius: float,
    width: float,
    web_radius: float,
    spoke_count: int,
) -> MeshGeometry:
    geom = MeshGeometry()
    profile = _tooth_profile(teeth, root_radius, outer_radius)
    bottom_outer, bottom_inner = _ring_indices(
        geom,
        z=-width * 0.5,
        outer_profile=profile,
        inner_radius=inner_radius,
    )
    top_outer, top_inner = _ring_indices(
        geom,
        z=width * 0.5,
        outer_profile=profile,
        inner_radius=inner_radius,
    )
    count = len(profile)
    for i in range(count):
        j = (i + 1) % count
        _add_quad(geom, bottom_outer[i], bottom_outer[j], top_outer[j], top_outer[i])
        _add_quad(geom, bottom_inner[j], bottom_inner[i], top_inner[i], top_inner[j])
        _add_quad(geom, top_inner[i], top_inner[j], top_outer[j], top_outer[i])
        _add_quad(geom, bottom_inner[j], bottom_inner[i], bottom_outer[i], bottom_outer[j])

    # Raised central hub bands and simple spoke pads make the procedural mesh read
    # as a machined spur gear without the cost of exact involute CAD surfaces.
    if web_radius > inner_radius:
        hub_profile = [web_radius for _ in range(count)]
        hub_bottom, _ = _ring_indices(geom, z=-width * 0.62, outer_profile=hub_profile, inner_radius=inner_radius)
        hub_top, _ = _ring_indices(geom, z=width * 0.62, outer_profile=hub_profile, inner_radius=inner_radius)
        for i in range(count):
            j = (i + 1) % count
            _add_quad(geom, hub_bottom[i], hub_bottom[j], hub_top[j], hub_top[i])
    if spoke_count:
        for spoke in range(spoke_count):
            angle = 2.0 * math.pi * spoke / spoke_count
            width_angle = 0.10
            r0 = web_radius * 0.85
            r1 = root_radius * 0.83
            z = width * 0.56
            pts = []
            for da, rr in ((-width_angle, r0), (-width_angle, r1), (width_angle, r1), (width_angle, r0)):
                pts.append(geom.add_vertex(rr * math.cos(angle + da), rr * math.sin(angle + da), z))
            _add_quad(geom, pts[0], pts[1], pts[2], pts[3])
    return geom


def _bevel_ring_geometry(
    *,
    teeth: int,
    root_radius: float,
    outer_radius: float,
    inner_radius: float,
    width: float,
    taper: float,
) -> MeshGeometry:
    geom = MeshGeometry()
    large_profile = _tooth_profile(teeth, root_radius, outer_radius)
    small_profile = [radius * taper for radius in large_profile]
    bottom_outer, bottom_inner = _ring_indices(
        geom,
        z=-width * 0.5,
        outer_profile=large_profile,
        inner_radius=inner_radius,
    )
    top_outer, top_inner = _ring_indices(
        geom,
        z=width * 0.5,
        outer_profile=small_profile,
        inner_radius=inner_radius * 0.92,
    )
    count = len(large_profile)
    for i in range(count):
        j = (i + 1) % count
        _add_quad(geom, bottom_outer[i], bottom_outer[j], top_outer[j], top_outer[i])
        _add_quad(geom, bottom_inner[j], bottom_inner[i], top_inner[i], top_inner[j])
        _add_quad(geom, top_inner[i], top_inner[j], top_outer[j], top_outer[i])
        _add_quad(geom, bottom_inner[j], bottom_inner[i], bottom_outer[i], bottom_outer[j])
    return geom


def _spur_mesh(
    *,
    module: float,
    teeth: int,
    width: float,
    name: str,
    bore: float,
    hub_d: float,
    hub_length: float,
    spokes: int | None = None,
):
    pitch_radius = 0.5 * module * teeth
    return mesh_from_geometry(
        _toothed_ring_geometry(
            teeth=teeth,
            root_radius=max(hub_d * 0.55, pitch_radius - 1.15 * module),
            outer_radius=pitch_radius + module,
            inner_radius=bore * 0.5,
            width=width,
            web_radius=max(hub_d * 0.65, pitch_radius * 0.45),
            spoke_count=spokes or 0,
        ),
        name,
    )


def _bevel_mesh(*, module: float, teeth: int, cone_angle: float, face_width: float, name: str):
    pitch_radius = 0.5 * module * teeth
    taper = 0.62 if cone_angle > 45.0 else 0.72
    return mesh_from_geometry(
        _bevel_ring_geometry(
            teeth=teeth,
            root_radius=pitch_radius - 1.05 * module,
            outer_radius=pitch_radius + module,
            inner_radius=0.016,
            width=face_width,
            taper=taper,
        ),
        name,
    )


def _horizontal_gear_visual(
    part,
    mesh,
    *,
    y: float,
    material: Material,
    name: str,
    phase: float = 0.0,
) -> None:
    part.visual(
        mesh,
        origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, phase)),
        material=material,
        name=name,
    )


def _vertical_gear_visual(part, mesh, *, z: float, material: Material, name: str) -> None:
    part.visual(mesh, origin=Origin(xyz=(0.0, 0.0, z)), material=material, name=name)


def _add_key_bar(
    part,
    *,
    axis: str,
    length: float,
    material: Material,
    name: str,
    offset: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    if axis == "y":
        size = (0.014, length, 0.010)
        xyz = (offset[0], offset[1], offset[2] + 0.033)
    elif axis == "z":
        size = (0.014, 0.010, length)
        xyz = (offset[0] + 0.044, offset[1], offset[2])
    else:
        size = (length, 0.014, 0.010)
        xyz = (offset[0], offset[1], offset[2] + 0.044)
    _add_box(part, size=size, xyz=xyz, material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gear_assemblies_study")

    frame_mat = model.material("shot_blasted_frame", rgba=(0.33, 0.35, 0.35, 1.0))
    dark_mat = model.material("black_oxide", rgba=(0.055, 0.060, 0.062, 1.0))
    shaft_mat = model.material("ground_shaft_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    gear_mat = model.material("oiled_machined_steel", rgba=(0.48, 0.47, 0.42, 1.0))
    brass_mat = model.material("bronze_wear_surfaces", rgba=(0.74, 0.54, 0.28, 1.0))
    cover_mat = model.material("brushed_cover_plate", rgba=(0.42, 0.46, 0.47, 1.0))
    seam_mat = model.material("dark_split_lines", rgba=(0.015, 0.016, 0.017, 1.0))

    bearing_ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.0305, tube=0.0125, radial_segments=20, tubular_segments=32),
        "split_bearing_ring",
    )
    clevis_mesh = mesh_from_geometry(
        ClevisBracketGeometry(
            (0.115, 0.070, 0.105),
            gap_width=0.047,
            bore_diameter=0.018,
            bore_center_z=0.069,
            base_thickness=0.020,
            corner_radius=0.004,
        ),
        "torque_arm_clevis",
    )
    input_spur = _spur_mesh(
        module=0.008,
        teeth=30,
        width=0.050,
        name="input_30t_spur",
        bore=0.034,
        hub_d=0.070,
        hub_length=0.074,
        spokes=5,
    )
    compound_large = _spur_mesh(
        module=0.008,
        teeth=42,
        width=0.055,
        name="compound_42t_spur",
        bore=0.034,
        hub_d=0.082,
        hub_length=0.080,
        spokes=6,
    )
    compound_pinion = _spur_mesh(
        module=0.008,
        teeth=18,
        width=0.046,
        name="compound_18t_pinion",
        bore=0.034,
        hub_d=0.062,
        hub_length=0.066,
        spokes=None,
    )
    transfer_spur = _spur_mesh(
        module=0.008,
        teeth=36,
        width=0.052,
        name="transfer_36t_spur",
        bore=0.034,
        hub_d=0.078,
        hub_length=0.076,
        spokes=6,
    )
    bevel_pinion_mesh = _bevel_mesh(
        module=0.0048,
        teeth=18,
        cone_angle=38.0,
        face_width=0.026,
        name="bevel_pinion_18t",
    )
    bevel_gear_mesh = _bevel_mesh(
        module=0.0048,
        teeth=28,
        cone_angle=52.0,
        face_width=0.027,
        name="bevel_output_28t",
    )

    frame = model.part("frame")
    _add_box(frame, size=(1.360, 0.740, 0.040), xyz=(-0.100, 0.000, 0.020), material=frame_mat, name="ground_base")
    _add_box(frame, size=(1.270, 0.045, 0.090), xyz=(-0.100, -0.340, 0.085), material=frame_mat, name="near_lower_rail")
    _add_box(frame, size=(1.270, 0.045, 0.090), xyz=(-0.100, 0.340, 0.085), material=frame_mat, name="far_lower_rail")
    _add_box(frame, size=(1.270, 0.030, 0.050), xyz=(-0.100, -0.340, 0.535), material=frame_mat, name="near_upper_rail")
    _add_box(frame, size=(1.270, 0.030, 0.050), xyz=(-0.100, 0.340, 0.535), material=frame_mat, name="far_upper_rail")
    for index, x in enumerate((-0.620, -0.360, -0.100, 0.130, 0.390)):
        _add_box(frame, size=(0.040, 0.045, 0.495), xyz=(x, -0.340, 0.287), material=frame_mat, name=f"near_post_{index}")
        _add_box(frame, size=(0.040, 0.045, 0.495), xyz=(x, 0.340, 0.287), material=frame_mat, name=f"far_post_{index}")
    for index, x in enumerate((-0.600, -0.220, 0.170, 0.440)):
        _add_box(frame, size=(0.045, 0.600, 0.060), xyz=(x, 0.000, 0.070), material=frame_mat, name=f"cross_tie_{index}")

    # Cover guide rails and bolted split-housing lips on the near side.
    _add_box(frame, size=(0.470, 0.025, 0.026), xyz=(-0.280, -0.370, 0.445), material=frame_mat, name="spur_cover_top_guide")
    _add_box(frame, size=(0.470, 0.025, 0.026), xyz=(-0.280, -0.370, 0.230), material=frame_mat, name="spur_cover_bottom_guide")
    _add_box(frame, size=(0.310, 0.025, 0.026), xyz=(0.170, -0.370, 0.445), material=frame_mat, name="bevel_cover_top_guide")
    _add_box(frame, size=(0.310, 0.025, 0.026), xyz=(0.170, -0.370, 0.230), material=frame_mat, name="bevel_cover_bottom_guide")

    horizontal_bearings = {
        "input": (-0.420, 0.340, (-0.250, 0.250)),
        "compound": (-0.100, 0.340, (-0.250, 0.250)),
        "transfer": (0.145, 0.340, (-0.250, 0.035)),
    }
    for shaft_name, (x, z, y_positions) in horizontal_bearings.items():
        for bearing_index, y in enumerate(y_positions):
            pedestal_height = z - 0.052 - 0.040 + 0.010
            _add_box(
                frame,
                size=(0.105, 0.060, pedestal_height),
                xyz=(x, y, 0.040 + pedestal_height / 2.0),
                material=frame_mat,
                name=f"{shaft_name}_pedestal_{bearing_index}",
            )
            frame.visual(
                bearing_ring_mesh,
                origin=Origin(xyz=(x, y, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=brass_mat,
                name=f"{shaft_name}_bearing_{bearing_index}_ring",
            )
            _add_box(
                frame,
                size=(0.122, 0.060, 0.034),
                xyz=(x, y, z + 0.055),
                material=frame_mat,
                name=f"{shaft_name}_bearing_{bearing_index}_cap",
            )
            _add_box(
                frame,
                size=(0.120, 0.006, 0.010),
                xyz=(x, y - 0.029, z + 0.038),
                material=seam_mat,
                name=f"{shaft_name}_bearing_{bearing_index}_split_line",
            )
            for bolt_index, bx in enumerate((-0.036, 0.036)):
                _cyl_on_axis(
                    frame,
                    radius=0.006,
                    length=0.014,
                    axis="z",
                    xyz=(x + bx, y, z + 0.079),
                    material=dark_mat,
                    name=f"{shaft_name}_bearing_{bearing_index}_bolt_{bolt_index}",
                )

    # Vertical output shaft support: lower thrust bearing, top steady bearing, and torque-arm bracket.
    output_x = 0.145
    output_y = 0.245
    _add_box(frame, size=(0.145, 0.145, 0.105), xyz=(output_x, output_y, 0.092), material=frame_mat, name="output_thrust_pedestal")
    frame.visual(
        bearing_ring_mesh,
        origin=Origin(xyz=(output_x, output_y, 0.185)),
        material=brass_mat,
        name="output_lower_bearing_ring",
    )
    _cyl_on_axis(
        frame,
        radius=0.028,
        length=0.115,
        axis="z",
        xyz=(output_x, output_y, 0.238),
        material=brass_mat,
        name="output_bearing_sleeve",
    )
    _add_box(frame, size=(0.165, 0.165, 0.030), xyz=(output_x, output_y, 0.238), material=frame_mat, name="output_bearing_bridge")
    _add_box(frame, size=(0.165, 0.032, 0.028), xyz=(output_x, output_y - 0.064, 0.238), material=frame_mat, name="output_cap_near_lip")
    _add_box(frame, size=(0.165, 0.032, 0.028), xyz=(output_x, output_y + 0.064, 0.238), material=frame_mat, name="output_cap_far_lip")
    _add_box(frame, size=(0.032, 0.096, 0.028), xyz=(output_x - 0.066, output_y, 0.238), material=frame_mat, name="output_cap_side_0")
    _add_box(frame, size=(0.032, 0.096, 0.028), xyz=(output_x + 0.066, output_y, 0.238), material=frame_mat, name="output_cap_side_1")
    _add_box(frame, size=(0.145, 0.006, 0.010), xyz=(output_x, output_y - 0.080, 0.238), material=seam_mat, name="output_split_line")
    _add_box(frame, size=(0.085, 0.075, 0.386), xyz=(output_x, output_y + 0.120, 0.323), material=frame_mat, name="output_torque_web")
    frame.visual(
        clevis_mesh,
        origin=Origin(xyz=(output_x, output_y + 0.215, 0.186), rpy=(0.0, 0.0, math.pi / 2.0)),
        material=frame_mat,
        name="torque_arm_clevis",
    )
    for bolt_index, bx in enumerate((-0.050, 0.050)):
        for by in (-0.050, 0.050):
            _cyl_on_axis(
                frame,
                radius=0.006,
                length=0.016,
                axis="z",
                xyz=(output_x + bx, output_y + by, 0.260),
                material=dark_mat,
                name=f"output_cap_bolt_{bolt_index}_{by:+.2f}",
            )

    input_shaft = model.part("input_shaft")
    _cyl_on_axis(input_shaft, radius=0.018, length=0.590, axis="y", xyz=(0.0, 0.0, 0.0), material=shaft_mat, name="shaft")
    _horizontal_gear_visual(input_shaft, input_spur, y=0.000, material=gear_mat, name="input_spur", phase=0.0)
    _add_key_bar(input_shaft, axis="y", length=0.088, material=dark_mat, name="input_key")
    _cyl_on_axis(input_shaft, radius=0.024, length=0.060, axis="y", xyz=(0.0, -0.325, 0.0), material=shaft_mat, name="input_coupler_stub")

    compound_shaft = model.part("compound_shaft")
    _cyl_on_axis(compound_shaft, radius=0.018, length=0.590, axis="y", xyz=(0.0, 0.0, 0.0), material=shaft_mat, name="shaft")
    _horizontal_gear_visual(compound_shaft, compound_large, y=0.000, material=gear_mat, name="large_spur", phase=math.pi / 42.0)
    _horizontal_gear_visual(compound_shaft, compound_pinion, y=-0.095, material=gear_mat, name="pinion_spur", phase=0.0)
    _add_key_bar(compound_shaft, axis="y", length=0.175, material=dark_mat, name="compound_key", offset=(0.0, -0.045, 0.0))

    transfer_shaft = model.part("transfer_shaft")
    _cyl_on_axis(transfer_shaft, radius=0.018, length=0.430, axis="y", xyz=(0.0, -0.090, 0.0), material=shaft_mat, name="shaft")
    _horizontal_gear_visual(transfer_shaft, transfer_spur, y=-0.095, material=gear_mat, name="transfer_spur", phase=math.pi / 36.0)
    _horizontal_gear_visual(transfer_shaft, bevel_pinion_mesh, y=0.090, material=gear_mat, name="bevel_pinion", phase=0.0)
    _add_key_bar(transfer_shaft, axis="y", length=0.182, material=dark_mat, name="transfer_key", offset=(0.0, -0.010, 0.0))

    output_shaft = model.part("output_shaft")
    _cyl_on_axis(output_shaft, radius=0.0186, length=0.560, axis="z", xyz=(0.0, 0.0, 0.130), material=shaft_mat, name="shaft")
    _vertical_gear_visual(output_shaft, bevel_gear_mesh, z=0.000, material=gear_mat, name="bevel_gear")
    _cyl_on_axis(output_shaft, radius=0.046, length=0.058, axis="z", xyz=(0.0, 0.0, 0.275), material=shaft_mat, name="keyed_output_hub")
    _add_key_bar(output_shaft, axis="z", length=0.110, material=dark_mat, name="output_key", offset=(0.0, 0.0, 0.275))
    _cyl_on_axis(output_shaft, radius=0.072, length=0.016, axis="z", xyz=(0.0, 0.0, 0.392), material=shaft_mat, name="output_flange")
    for angle_index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        _cyl_on_axis(
            output_shaft,
            radius=0.006,
            length=0.022,
            axis="z",
            xyz=(0.049 * math.cos(angle), 0.049 * math.sin(angle), 0.407),
            material=dark_mat,
            name=f"flange_bolt_{angle_index}",
        )

    spur_cover = model.part("spur_cover")
    _add_box(spur_cover, size=(0.465, 0.012, 0.205), xyz=(0.0, 0.0, 0.0), material=cover_mat, name="cover_plate")
    _add_box(spur_cover, size=(0.430, 0.010, 0.012), xyz=(0.0, 0.010, 0.088), material=dark_mat, name="upper_slide_tongue")
    _add_box(spur_cover, size=(0.430, 0.010, 0.012), xyz=(0.0, 0.010, -0.088), material=dark_mat, name="lower_slide_tongue")
    _add_box(spur_cover, size=(0.150, 0.018, 0.018), xyz=(0.0, -0.014, 0.000), material=dark_mat, name="pull_handle")
    for index, x in enumerate((-0.190, -0.095, 0.095, 0.190)):
        _cyl_on_axis(spur_cover, radius=0.005, length=0.014, axis="y", xyz=(x, -0.006, 0.075), material=dark_mat, name=f"cover_bolt_top_{index}")
        _cyl_on_axis(spur_cover, radius=0.005, length=0.014, axis="y", xyz=(x, -0.006, -0.075), material=dark_mat, name=f"cover_bolt_bottom_{index}")

    bevel_cover = model.part("bevel_cover")
    _add_box(bevel_cover, size=(0.305, 0.012, 0.205), xyz=(0.0, 0.0, 0.0), material=cover_mat, name="cover_plate")
    _add_box(bevel_cover, size=(0.270, 0.010, 0.012), xyz=(0.0, 0.010, 0.088), material=dark_mat, name="upper_slide_tongue")
    _add_box(bevel_cover, size=(0.270, 0.010, 0.012), xyz=(0.0, 0.010, -0.088), material=dark_mat, name="lower_slide_tongue")
    _add_box(bevel_cover, size=(0.105, 0.018, 0.018), xyz=(0.0, -0.014, 0.000), material=dark_mat, name="pull_handle")
    for index, x in enumerate((-0.115, 0.0, 0.115)):
        _cyl_on_axis(bevel_cover, radius=0.005, length=0.014, axis="y", xyz=(x, -0.006, 0.075), material=dark_mat, name=f"cover_bolt_top_{index}")
        _cyl_on_axis(bevel_cover, radius=0.005, length=0.014, axis="y", xyz=(x, -0.006, -0.075), material=dark_mat, name=f"cover_bolt_bottom_{index}")

    model.articulation(
        "frame_to_input_shaft",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=input_shaft,
        origin=Origin(xyz=(-0.420, 0.000, 0.340)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=18.0),
    )
    model.articulation(
        "frame_to_compound_shaft",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=compound_shaft,
        origin=Origin(xyz=(-0.100, 0.000, 0.340)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=12.0),
        mimic=Mimic(joint="frame_to_input_shaft", multiplier=-(30.0 / 42.0)),
    )
    model.articulation(
        "frame_to_transfer_shaft",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=transfer_shaft,
        origin=Origin(xyz=(0.145, 0.000, 0.340)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=9.0),
        mimic=Mimic(joint="frame_to_compound_shaft", multiplier=-(18.0 / 36.0)),
    )
    model.articulation(
        "frame_to_output_shaft",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=output_shaft,
        origin=Origin(xyz=(output_x, output_y, 0.340)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=150.0, velocity=7.0),
        mimic=Mimic(joint="frame_to_transfer_shaft", multiplier=-(18.0 / 28.0)),
    )
    model.articulation(
        "frame_to_spur_cover",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=spur_cover,
        origin=Origin(xyz=(-0.280, -0.388, 0.338)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.15, lower=0.0, upper=0.090),
    )
    model.articulation(
        "frame_to_bevel_cover",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=bevel_cover,
        origin=Origin(xyz=(0.170, -0.388, 0.338)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.15, lower=0.0, upper=0.080),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    input_shaft = object_model.get_part("input_shaft")
    compound_shaft = object_model.get_part("compound_shaft")
    transfer_shaft = object_model.get_part("transfer_shaft")
    output_shaft = object_model.get_part("output_shaft")

    ctx.allow_overlap(
        frame,
        input_shaft,
        reason="The input shaft is intentionally captured inside the split bronze bearing proxies.",
    )
    ctx.allow_overlap(
        frame,
        compound_shaft,
        reason="The compound shaft is intentionally captured inside the split bronze bearing proxies.",
    )
    ctx.allow_overlap(
        frame,
        transfer_shaft,
        reason="The transfer shaft is intentionally captured inside the split bronze bearing proxies.",
    )
    ctx.allow_overlap(
        frame,
        output_shaft,
        reason="The vertical output shaft is intentionally captured inside the thrust bearing proxy.",
    )

    ctx.expect_origin_distance(
        input_shaft,
        compound_shaft,
        axes="x",
        min_dist=0.305,
        max_dist=0.335,
        name="first spur mesh center distance",
    )
    ctx.expect_origin_distance(
        compound_shaft,
        transfer_shaft,
        axes="x",
        min_dist=0.235,
        max_dist=0.255,
        name="second spur mesh center distance",
    )
    ctx.expect_origin_distance(
        transfer_shaft,
        output_shaft,
        axes="xy",
        min_dist=0.235,
        max_dist=0.255,
        name="bevel stage offset is compact",
    )
    ctx.expect_overlap(
        input_shaft,
        frame,
        axes="xyz",
        min_overlap=0.020,
        name="input shaft retained by bearing blocks",
    )
    ctx.expect_overlap(
        compound_shaft,
        frame,
        axes="xyz",
        min_overlap=0.020,
        name="compound shaft retained by bearing blocks",
    )
    ctx.expect_overlap(
        transfer_shaft,
        frame,
        axes="xyz",
        min_overlap=0.020,
        name="transfer shaft retained by bearing blocks",
    )
    ctx.expect_overlap(
        output_shaft,
        frame,
        axes="xyz",
        min_overlap=0.010,
        name="output shaft retained by thrust bearing",
    )

    spur_cover = object_model.get_part("spur_cover")
    bevel_cover = object_model.get_part("bevel_cover")
    spur_cover_joint = object_model.get_articulation("frame_to_spur_cover")
    bevel_cover_joint = object_model.get_articulation("frame_to_bevel_cover")
    spur_rest = ctx.part_world_position(spur_cover)
    bevel_rest = ctx.part_world_position(bevel_cover)
    with ctx.pose({spur_cover_joint: 0.080, bevel_cover_joint: 0.070}):
        spur_pulled = ctx.part_world_position(spur_cover)
        bevel_pulled = ctx.part_world_position(bevel_cover)
    ctx.check(
        "spur cover slides out along guide",
        spur_rest is not None and spur_pulled is not None and spur_pulled[1] < spur_rest[1] - 0.070,
        details=f"rest={spur_rest}, pulled={spur_pulled}",
    )
    ctx.check(
        "bevel cover slides out along guide",
        bevel_rest is not None and bevel_pulled is not None and bevel_pulled[1] < bevel_rest[1] - 0.060,
        details=f"rest={bevel_rest}, pulled={bevel_pulled}",
    )

    return ctx.report()


object_model = build_object_model()
