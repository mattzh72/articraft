from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _yz_section(
    x_pos: float,
    *,
    width: float,
    height: float,
    radius: float,
    z_center: float,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y, z + z_center)
        for z, y in rounded_rect_profile(height, width, radius, corner_segments=corner_segments)
    ]


def _axial_cylinder(
    *,
    radius: float,
    length: float,
    axis: str,
    center: tuple[float, float, float],
    radial_segments: int = 28,
):
    geom = CylinderGeometry(radius=radius, height=length, radial_segments=radial_segments)
    if axis == "x":
        geom.rotate_y(pi / 2.0)
    elif axis == "y":
        geom.rotate_x(pi / 2.0)
    elif axis != "z":
        raise ValueError(f"unsupported axis: {axis}")
    geom.translate(*center)
    return geom


def _build_body_mesh():
    body = section_loft(
        [
            _yz_section(-0.19, width=0.20, height=0.18, radius=0.038, z_center=0.133),
            _yz_section(-0.05, width=0.24, height=0.22, radius=0.048, z_center=0.133),
            _yz_section(0.10, width=0.24, height=0.21, radius=0.046, z_center=0.130),
            _yz_section(0.24, width=0.16, height=0.16, radius=0.032, z_center=0.128),
        ]
    )
    body.merge(BoxGeometry((0.42, 0.20, 0.03)).translate(0.0, 0.0, 0.015))
    body.merge(BoxGeometry((0.07, 0.18, 0.04)).translate(-0.205, 0.0, 0.020))
    body.merge(BoxGeometry((0.16, 0.006, 0.12)).translate(0.0, 0.117, 0.130))
    body.merge(BoxGeometry((0.16, 0.006, 0.12)).translate(0.0, -0.117, 0.130))
    body.merge(BoxGeometry((0.19, 0.12, 0.006)).translate(-0.020, 0.0, 0.242))
    body.merge(_axial_cylinder(radius=0.018, length=0.024, axis="y", center=(-0.110, 0.108, 0.070)))
    body.merge(_axial_cylinder(radius=0.018, length=0.024, axis="y", center=(-0.110, -0.108, 0.070)))
    return body


def _build_side_hatch_mesh():
    hatch = BoxGeometry((0.150, 0.004, 0.106))
    hatch.merge(BoxGeometry((0.120, 0.008, 0.018)).translate(0.0, 0.0, -0.030))
    hatch.merge(BoxGeometry((0.060, 0.008, 0.040)).translate(-0.038, 0.0, 0.000))
    hatch.merge(_axial_cylinder(radius=0.006, length=0.030, axis="x", center=(-0.040, 0.0, 0.058)))
    hatch.merge(_axial_cylinder(radius=0.006, length=0.030, axis="x", center=(0.040, 0.0, 0.058)))
    for x_pos in (-0.056, 0.056):
        for z_pos in (-0.036, 0.036):
            hatch.merge(_axial_cylinder(radius=0.005, length=0.004, axis="y", center=(x_pos, 0.003, z_pos)))
    return hatch


def _build_top_hatch_mesh():
    hatch = BoxGeometry((0.180, 0.115, 0.005))
    hatch.merge(BoxGeometry((0.125, 0.040, 0.012)).translate(0.012, 0.0, 0.008))
    hatch.merge(BoxGeometry((0.055, 0.018, 0.014)).translate(0.067, 0.0, 0.010))
    hatch.merge(_axial_cylinder(radius=0.006, length=0.040, axis="y", center=(-0.085, -0.028, 0.004)))
    hatch.merge(_axial_cylinder(radius=0.006, length=0.040, axis="y", center=(-0.085, 0.028, 0.004)))
    for x_pos in (-0.070, 0.070):
        for y_pos in (-0.042, 0.042):
            hatch.merge(_axial_cylinder(radius=0.005, length=0.005, axis="z", center=(x_pos, y_pos, 0.005)))
    return hatch


def _build_adapter_mesh():
    adapter = _axial_cylinder(radius=0.038, length=0.016, axis="x", center=(0.008, 0.0, 0.0))
    adapter.merge(_axial_cylinder(radius=0.022, length=0.050, axis="x", center=(0.030, 0.0, 0.0)))
    adapter.merge(BoxGeometry((0.032, 0.052, 0.034)).translate(0.022, 0.0, 0.0))
    adapter.merge(BoxGeometry((0.028, 0.014, 0.032)).translate(0.010, 0.032, 0.0))
    adapter.merge(BoxGeometry((0.028, 0.014, 0.032)).translate(0.010, -0.032, 0.0))
    adapter.merge(_axial_cylinder(radius=0.022, length=0.016, axis="y", center=(0.030, 0.029, 0.0)))
    adapter.merge(_axial_cylinder(radius=0.022, length=0.016, axis="y", center=(0.030, -0.029, 0.0)))
    adapter.merge(_axial_cylinder(radius=0.006, length=0.004, axis="x", center=(0.016, 0.035, 0.0)))
    adapter.merge(_axial_cylinder(radius=0.006, length=0.004, axis="x", center=(0.016, -0.035, 0.0)))
    return adapter


def _build_lower_wand_mesh():
    wand = tube_from_spline_points(
        [
            (0.015, 0.0, 0.000),
            (0.105, 0.0, -0.006),
            (0.220, 0.0, -0.022),
            (0.320, 0.0, -0.040),
        ],
        radius=0.015,
        samples_per_segment=16,
        radial_segments=18,
    )
    wand.merge(_axial_cylinder(radius=0.018, length=0.042, axis="y", center=(0.0, 0.0, 0.0)))
    wand.merge(_axial_cylinder(radius=0.023, length=0.040, axis="x", center=(0.040, 0.0, -0.003)))
    wand.merge(BoxGeometry((0.058, 0.030, 0.024)).translate(0.038, 0.0, -0.010))
    wand.merge(BoxGeometry((0.040, 0.050, 0.032)).translate(0.320, 0.0, -0.040))
    wand.merge(_axial_cylinder(radius=0.020, length=0.016, axis="y", center=(0.340, 0.029, -0.045)))
    wand.merge(_axial_cylinder(radius=0.020, length=0.016, axis="y", center=(0.340, -0.029, -0.045)))
    wand.merge(_axial_cylinder(radius=0.021, length=0.024, axis="x", center=(0.304, 0.0, -0.036)))
    return wand


def _build_upper_wand_mesh():
    wand = tube_from_spline_points(
        [
            (0.015, 0.0, 0.000),
            (0.102, 0.0, -0.010),
            (0.215, 0.0, -0.028),
            (0.300, 0.0, -0.050),
        ],
        radius=0.014,
        samples_per_segment=16,
        radial_segments=18,
    )
    wand.merge(_axial_cylinder(radius=0.017, length=0.042, axis="y", center=(0.0, 0.0, 0.0)))
    wand.merge(_axial_cylinder(radius=0.022, length=0.044, axis="x", center=(0.040, 0.0, -0.004)))
    wand.merge(_axial_cylinder(radius=0.023, length=0.085, axis="x", center=(0.190, 0.0, -0.020)))
    wand.merge(BoxGeometry((0.040, 0.050, 0.030)).translate(0.300, 0.0, -0.048))
    wand.merge(_axial_cylinder(radius=0.019, length=0.014, axis="y", center=(0.320, 0.028, -0.055)))
    wand.merge(_axial_cylinder(radius=0.019, length=0.014, axis="y", center=(0.320, -0.028, -0.055)))
    return wand


def _build_floor_nozzle_mesh():
    nozzle = _axial_cylinder(radius=0.016, length=0.038, axis="y", center=(0.0, 0.0, 0.0))
    nozzle.merge(BoxGeometry((0.050, 0.040, 0.030)).translate(0.026, 0.0, -0.010))
    nozzle.merge(
        tube_from_spline_points(
            [
                (0.012, 0.0, -0.002),
                (0.046, 0.0, -0.018),
                (0.085, 0.0, -0.028),
            ],
            radius=0.012,
            samples_per_segment=14,
            radial_segments=18,
        )
    )
    nozzle.merge(BoxGeometry((0.290, 0.096, 0.022)).translate(0.145, 0.0, -0.044))
    nozzle.merge(BoxGeometry((0.180, 0.080, 0.026)).translate(0.155, 0.0, -0.029))
    nozzle.merge(BoxGeometry((0.290, 0.008, 0.010)).translate(0.145, 0.044, -0.048))
    nozzle.merge(BoxGeometry((0.290, 0.008, 0.010)).translate(0.145, -0.044, -0.048))
    nozzle.merge(BoxGeometry((0.036, 0.080, 0.020)).translate(0.278, 0.0, -0.040))
    nozzle.merge(BoxGeometry((0.045, 0.080, 0.016)).translate(0.040, 0.0, -0.050))
    return nozzle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_vacuum_cleaner")

    body_enamel = model.material("body_enamel", rgba=(0.38, 0.43, 0.31, 1.0))
    service_cream = model.material("service_cream", rgba=(0.82, 0.80, 0.72, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.18, 0.19, 0.20, 1.0))
    steel = model.material("steel", rgba=(0.67, 0.69, 0.71, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    body = model.part("body")
    body.visual(_save_mesh("vacuum_body_shell", _build_body_mesh()), material=body_enamel, name="body_shell")
    body.inertial = Inertial.from_geometry(
        Box((0.46, 0.25, 0.28)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
    )

    wheel_visual_origin = Origin(rpy=(pi / 2.0, 0.0, 0.0))
    for part_name, side_sign in (("left_wheel", 1.0), ("right_wheel", -1.0)):
        wheel = model.part(part_name)
        wheel.visual(
            Cylinder(radius=0.072, length=0.032),
            origin=wheel_visual_origin,
            material=rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.047, length=0.024),
            origin=wheel_visual_origin,
            material=dark_graphite,
            name="hub",
        )
        wheel.visual(
            Cylinder(radius=0.022, length=0.010),
            origin=Origin(xyz=(0.0, side_sign * 0.011, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="cap",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.072, length=0.032),
            mass=0.75,
            origin=wheel_visual_origin,
        )

    left_hatch = model.part("left_hatch")
    left_hatch.visual(
        Box((0.120, 0.004, 0.098)),
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
        material=service_cream,
        name="left_service_hatch",
    )
    left_hatch.visual(
        Box((0.096, 0.006, 0.018)),
        origin=Origin(xyz=(0.032, 0.0, -0.028)),
        material=service_cream,
        name="left_hatch_rib",
    )
    left_hatch.visual(
        Box((0.046, 0.010, 0.028)),
        origin=Origin(xyz=(-0.010, 0.003, 0.000)),
        material=service_cream,
        name="left_hatch_grip",
    )
    left_hatch.visual(
        Cylinder(radius=0.006, length=0.088),
        origin=Origin(xyz=(-0.018, 0.0, 0.048), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="left_hatch_hinge",
    )
    for x_pos in (0.000, 0.080):
        for z_pos in (-0.032, 0.032):
            left_hatch.visual(
                Cylinder(radius=0.005, length=0.006),
                origin=Origin(xyz=(x_pos, 0.003, z_pos)),
                material=steel,
                name=f"left_hatch_bolt_{int((x_pos + 0.01) * 1000)}_{int((z_pos + 0.04) * 1000)}",
            )
    left_hatch.inertial = Inertial.from_geometry(Box((0.15, 0.012, 0.11)), mass=0.18)

    right_hatch = model.part("right_hatch")
    right_hatch.visual(
        Box((0.120, 0.004, 0.098)),
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
        material=service_cream,
        name="right_service_hatch",
    )
    right_hatch.visual(
        Box((0.096, 0.006, 0.018)),
        origin=Origin(xyz=(0.032, 0.0, -0.028)),
        material=service_cream,
        name="right_hatch_rib",
    )
    right_hatch.visual(
        Box((0.046, 0.010, 0.028)),
        origin=Origin(xyz=(-0.010, 0.003, 0.000)),
        material=service_cream,
        name="right_hatch_grip",
    )
    right_hatch.visual(
        Cylinder(radius=0.006, length=0.088),
        origin=Origin(xyz=(-0.018, 0.0, 0.048), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="right_hatch_hinge",
    )
    for x_pos in (0.000, 0.080):
        for z_pos in (-0.032, 0.032):
            right_hatch.visual(
                Cylinder(radius=0.005, length=0.006),
                origin=Origin(xyz=(x_pos, 0.003, z_pos)),
                material=steel,
                name=f"right_hatch_bolt_{int((x_pos + 0.01) * 1000)}_{int((z_pos + 0.04) * 1000)}",
            )
    right_hatch.inertial = Inertial.from_geometry(Box((0.15, 0.012, 0.11)), mass=0.18)

    top_hatch = model.part("top_hatch")
    top_hatch.visual(
        Box((0.170, 0.110, 0.004)),
        origin=Origin(xyz=(0.000, 0.0, 0.002)),
        material=service_cream,
        name="top_service_hatch",
    )
    top_hatch.visual(
        Box((0.120, 0.040, 0.010)),
        origin=Origin(xyz=(0.008, 0.0, 0.009)),
        material=service_cream,
        name="top_hatch_handle_base",
    )
    top_hatch.visual(
        Box((0.050, 0.016, 0.012)),
        origin=Origin(xyz=(0.062, 0.0, 0.011)),
        material=steel,
        name="top_hatch_latch",
    )
    top_hatch.visual(
        Cylinder(radius=0.006, length=0.036),
        origin=Origin(xyz=(-0.072, -0.024, 0.004), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="top_hatch_left_hinge",
    )
    top_hatch.visual(
        Cylinder(radius=0.006, length=0.036),
        origin=Origin(xyz=(-0.072, 0.024, 0.004), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="top_hatch_right_hinge",
    )
    top_hatch.inertial = Inertial.from_geometry(Box((0.18, 0.12, 0.02)), mass=0.20)

    adapter = model.part("adapter_flange")
    adapter.visual(
        Box((0.018, 0.078, 0.078)),
        origin=Origin(xyz=(0.009, 0.0, 0.0)),
        material=steel,
        name="adapter_flange_plate",
    )
    adapter.visual(
        Cylinder(radius=0.022, length=0.040),
        origin=Origin(xyz=(0.038, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="adapter_neck",
    )
    adapter.visual(
        Box((0.032, 0.050, 0.026)),
        origin=Origin(xyz=(0.024, 0.0, 0.0)),
        material=steel,
        name="adapter_saddle",
    )
    adapter.visual(
        Box((0.022, 0.012, 0.034)),
        origin=Origin(xyz=(0.050, 0.028, 0.0)),
        material=dark_graphite,
        name="adapter_left_yoke",
    )
    adapter.visual(
        Box((0.022, 0.012, 0.034)),
        origin=Origin(xyz=(0.050, -0.028, 0.0)),
        material=dark_graphite,
        name="adapter_right_yoke",
    )
    adapter.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(0.012, 0.030, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_graphite,
        name="adapter_upper_bolt",
    )
    adapter.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(0.012, -0.030, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_graphite,
        name="adapter_lower_bolt",
    )
    adapter.inertial = Inertial.from_geometry(
        Box((0.07, 0.08, 0.06)),
        mass=0.30,
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
    )

    lower_wand = model.part("lower_wand")
    lower_wand.visual(
        Cylinder(radius=0.018, length=0.038),
        origin=Origin(xyz=(0.015, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_graphite,
        name="lower_wand_pivot_sleeve",
    )
    lower_wand.visual(
        Cylinder(radius=0.014, length=0.255),
        origin=Origin(xyz=(0.165, 0.0, -0.019), rpy=(0.0, (pi / 2.0) - 0.10, 0.0)),
        material=steel,
        name="lower_wand_tube",
    )
    lower_wand.visual(
        Box((0.055, 0.032, 0.024)),
        origin=Origin(xyz=(0.043, 0.0, -0.010)),
        material=steel,
        name="lower_wand_collar",
    )
    lower_wand.visual(
        Box((0.120, 0.024, 0.022)),
        origin=Origin(xyz=(0.255, 0.0, -0.032)),
        material=steel,
        name="lower_wand_rear_brace",
    )
    lower_wand.visual(
        Box((0.050, 0.036, 0.040)),
        origin=Origin(xyz=(0.332, 0.0, -0.043)),
        material=dark_graphite,
        name="lower_wand_yoke",
    )
    lower_wand.visual(
        Cylinder(radius=0.019, length=0.038),
        origin=Origin(xyz=(0.334, 0.0, -0.043), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_graphite,
        name="lower_wand_end_sleeve",
    )
    lower_wand.inertial = Inertial.from_geometry(
        Box((0.36, 0.07, 0.08)),
        mass=0.55,
        origin=Origin(xyz=(0.18, 0.0, -0.020)),
    )

    upper_wand = model.part("upper_wand")
    upper_wand.visual(
        Cylinder(radius=0.017, length=0.036),
        origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_graphite,
        name="upper_wand_pivot_sleeve",
    )
    upper_wand.visual(
        Box((0.048, 0.028, 0.022)),
        origin=Origin(xyz=(0.036, 0.0, -0.010)),
        material=steel,
        name="upper_wand_front_collar",
    )
    upper_wand.visual(
        Box((0.070, 0.024, 0.060)),
        origin=Origin(xyz=(0.050, 0.0, -0.020)),
        material=steel,
        name="upper_wand_transition_gusset",
    )
    upper_wand.visual(
        Cylinder(radius=0.013, length=0.235),
        origin=Origin(xyz=(0.150, 0.0, -0.029), rpy=(0.0, (pi / 2.0) - 0.15, 0.0)),
        material=steel,
        name="upper_wand_tube",
    )
    upper_wand.visual(
        Cylinder(radius=0.018, length=0.072),
        origin=Origin(xyz=(0.175, 0.0, -0.021), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="upper_wand_service_collar",
    )
    upper_wand.visual(
        Box((0.104, 0.022, 0.020)),
        origin=Origin(xyz=(0.256, 0.0, -0.041)),
        material=steel,
        name="upper_wand_rear_brace",
    )
    upper_wand.visual(
        Box((0.044, 0.034, 0.038)),
        origin=Origin(xyz=(0.312, 0.0, -0.055)),
        material=dark_graphite,
        name="upper_wand_yoke",
    )
    upper_wand.visual(
        Cylinder(radius=0.016, length=0.036),
        origin=Origin(xyz=(0.315, 0.0, -0.055), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_graphite,
        name="upper_wand_end_sleeve",
    )
    upper_wand.inertial = Inertial.from_geometry(
        Box((0.34, 0.07, 0.09)),
        mass=0.50,
        origin=Origin(xyz=(0.17, 0.0, -0.024)),
    )

    floor_nozzle = model.part("floor_nozzle")
    floor_nozzle.visual(
        Cylinder(radius=0.016, length=0.036),
        origin=Origin(xyz=(0.016, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_graphite,
        name="floor_nozzle_pivot_sleeve",
    )
    floor_nozzle.visual(
        Box((0.056, 0.040, 0.028)),
        origin=Origin(xyz=(0.040, 0.0, -0.014)),
        material=dark_graphite,
        name="floor_nozzle_neck",
    )
    floor_nozzle.visual(
        Box((0.060, 0.030, 0.024)),
        origin=Origin(xyz=(0.082, 0.0, -0.026)),
        material=dark_graphite,
        name="floor_nozzle_throat",
    )
    floor_nozzle.visual(
        Box((0.260, 0.094, 0.020)),
        origin=Origin(xyz=(0.190, 0.0, -0.045)),
        material=dark_graphite,
        name="floor_nozzle_base",
    )
    floor_nozzle.visual(
        Box((0.150, 0.074, 0.018)),
        origin=Origin(xyz=(0.170, 0.0, -0.031)),
        material=dark_graphite,
        name="floor_nozzle_upper_shell",
    )
    floor_nozzle.visual(
        Box((0.260, 0.008, 0.010)),
        origin=Origin(xyz=(0.190, 0.043, -0.048)),
        material=rubber,
        name="floor_nozzle_left_skid",
    )
    floor_nozzle.visual(
        Box((0.260, 0.008, 0.010)),
        origin=Origin(xyz=(0.190, -0.043, -0.048)),
        material=rubber,
        name="floor_nozzle_right_skid",
    )
    floor_nozzle.visual(
        Box((0.050, 0.078, 0.014)),
        origin=Origin(xyz=(0.035, 0.0, -0.052)),
        material=rubber,
        name="floor_nozzle_rear_bumper",
    )
    floor_nozzle.inertial = Inertial.from_geometry(
        Box((0.30, 0.10, 0.06)),
        mass=0.65,
        origin=Origin(xyz=(0.15, 0.0, -0.028)),
    )

    model.articulation(
        "body_to_left_wheel",
        ArticulationType.FIXED,
        parent=body,
        child="left_wheel",
        origin=Origin(xyz=(-0.110, 0.136, 0.070)),
    )
    model.articulation(
        "body_to_right_wheel",
        ArticulationType.FIXED,
        parent=body,
        child="right_wheel",
        origin=Origin(xyz=(-0.110, -0.136, 0.070)),
    )
    model.articulation(
        "body_to_left_hatch",
        ArticulationType.FIXED,
        parent=body,
        child=left_hatch,
        origin=Origin(xyz=(0.045, 0.1292, 0.130)),
    )
    model.articulation(
        "body_to_right_hatch",
        ArticulationType.FIXED,
        parent=body,
        child=right_hatch,
        origin=Origin(xyz=(0.045, -0.1292, 0.130), rpy=(0.0, 0.0, pi)),
    )
    model.articulation(
        "body_to_top_hatch",
        ArticulationType.FIXED,
        parent=body,
        child=top_hatch,
        origin=Origin(xyz=(-0.015, 0.0, 0.245)),
    )
    model.articulation(
        "body_to_adapter",
        ArticulationType.FIXED,
        parent=body,
        child=adapter,
        origin=Origin(xyz=(0.240, 0.0, 0.155)),
    )
    model.articulation(
        "adapter_to_lower_wand",
        ArticulationType.REVOLUTE,
        parent=adapter,
        child=lower_wand,
        origin=Origin(xyz=(0.061, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-0.35, upper=0.90),
    )
    model.articulation(
        "lower_to_upper_wand",
        ArticulationType.REVOLUTE,
        parent=lower_wand,
        child=upper_wand,
        origin=Origin(xyz=(0.356, 0.0, -0.043)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.6, lower=-0.85, upper=0.65),
    )
    model.articulation(
        "upper_to_floor_nozzle",
        ArticulationType.REVOLUTE,
        parent=upper_wand,
        child=floor_nozzle,
        origin=Origin(xyz=(0.334, 0.0, -0.055)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.50, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    left_hatch = object_model.get_part("left_hatch")
    right_hatch = object_model.get_part("right_hatch")
    top_hatch = object_model.get_part("top_hatch")
    adapter = object_model.get_part("adapter_flange")
    lower_wand = object_model.get_part("lower_wand")
    upper_wand = object_model.get_part("upper_wand")
    floor_nozzle = object_model.get_part("floor_nozzle")
    shoulder = object_model.get_articulation("adapter_to_lower_wand")
    elbow = object_model.get_articulation("lower_to_upper_wand")
    nozzle_pitch = object_model.get_articulation("upper_to_floor_nozzle")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(left_wheel, body, name="left wheel is carried by body boss")
    ctx.expect_contact(right_wheel, body, name="right wheel is carried by body boss")
    ctx.expect_contact(left_hatch, body, name="left service hatch seats on body shell")
    ctx.expect_contact(right_hatch, body, name="right service hatch seats on body shell")
    ctx.expect_contact(top_hatch, body, name="top service hatch seats on body shell")
    ctx.expect_contact(adapter, body, name="adapter flange bolts onto body nose")
    ctx.expect_contact(lower_wand, adapter, name="lower wand sits in adapter collar")
    ctx.expect_contact(upper_wand, lower_wand, name="upper wand sits in elbow collar")
    ctx.expect_contact(floor_nozzle, upper_wand, name="nozzle pivot sits in wand yoke")
    ctx.expect_gap(floor_nozzle, body, axis="x", min_gap=0.30, name="floor nozzle stays forward of canister body")

    nozzle_aabb = ctx.part_world_aabb(floor_nozzle)
    nozzle_near_floor = nozzle_aabb is not None and abs(nozzle_aabb[0][2]) <= 0.010
    ctx.check(
        "default pose keeps nozzle near floor",
        nozzle_near_floor,
        details=f"floor nozzle aabb={nozzle_aabb}",
    )

    with ctx.pose({shoulder: 0.0, elbow: 0.0, nozzle_pitch: 0.0}):
        parked_nozzle = ctx.part_world_position(floor_nozzle)
    with ctx.pose({shoulder: 0.55, elbow: 0.35, nozzle_pitch: 0.20}):
        lifted_nozzle = ctx.part_world_position(floor_nozzle)

    wand_lifts = (
        parked_nozzle is not None
        and lifted_nozzle is not None
        and lifted_nozzle[2] > parked_nozzle[2] + 0.18
    )
    ctx.check(
        "positive wand articulations lift the nozzle",
        wand_lifts,
        details=f"parked={parked_nozzle}, lifted={lifted_nozzle}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
