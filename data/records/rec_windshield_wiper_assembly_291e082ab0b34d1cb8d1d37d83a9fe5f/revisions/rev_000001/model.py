from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _translated_profile(profile, dx: float, dy: float):
    return [(x + dx, y + dy) for x, y in profile]


def _rubber_wedge(length: float, top_width: float, height: float) -> MeshGeometry:
    """Triangular squeegee prism extruded along local Y."""
    half_y = length / 2.0
    half_w = top_width / 2.0
    geom = MeshGeometry()
    verts = [
        (-half_w, -half_y, 0.0),
        (half_w, -half_y, 0.0),
        (0.0, -half_y, -height),
        (-half_w, half_y, 0.0),
        (half_w, half_y, 0.0),
        (0.0, half_y, -height),
    ]
    for x, y, z in verts:
        geom.add_vertex(x, y, z)

    # End caps.
    geom.add_face(0, 2, 1)
    geom.add_face(3, 4, 5)
    # Top face.
    geom.add_face(0, 1, 4)
    geom.add_face(0, 4, 3)
    # Sloped sides.
    geom.add_face(1, 2, 5)
    geom.add_face(1, 5, 4)
    geom.add_face(2, 0, 3)
    geom.add_face(2, 3, 5)
    return geom


def _lightened_arm() -> MeshGeometry:
    outer = [
        (0.030, -0.030),
        (0.145, -0.024),
        (0.575, -0.011),
        (0.575, 0.011),
        (0.145, 0.024),
        (0.030, 0.030),
    ]
    holes = [
        _translated_profile(rounded_rect_profile(0.170, 0.018, 0.008), 0.260, 0.0),
        _translated_profile(rounded_rect_profile(0.130, 0.014, 0.006), 0.455, 0.0),
    ]
    return ExtrudeWithHolesGeometry(outer, holes, 0.010, cap=True, center=True)


def _hub_cap() -> MeshGeometry:
    # Low domed, stamped cap rather than a plain cylinder.
    return LatheGeometry(
        [
            (0.000, 0.000),
            (0.048, 0.000),
            (0.056, 0.004),
            (0.048, 0.012),
            (0.027, 0.020),
            (0.000, 0.020),
        ],
        segments=56,
        closed=True,
    )


def _coil_spring() -> MeshGeometry:
    points = []
    start_x = 0.155
    length = 0.210
    turns = 8
    samples = turns * 16 + 1
    for i in range(samples):
        t = i / (samples - 1)
        angle = 2.0 * math.pi * turns * t
        points.append(
            (
                start_x + length * t,
                0.031 + 0.0045 * math.cos(angle),
                0.025 + 0.0045 * math.sin(angle),
            )
        )
    return tube_from_spline_points(
        points,
        radius=0.0013,
        samples_per_segment=2,
        radial_segments=8,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="windshield_wiper_assembly")

    satin_black = model.material("satin_black", rgba=(0.005, 0.006, 0.006, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.0, 0.0, 0.0, 1.0))
    dark_metal = model.material("dark_phosphate_metal", rgba=(0.025, 0.026, 0.024, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.60, 1.0))
    zinc = model.material("zinc_plated_fasteners", rgba=(0.78, 0.76, 0.68, 1.0))
    glass = model.material("tinted_windshield_glass", rgba=(0.12, 0.20, 0.26, 0.38))

    mount = model.part("mount")
    mount.visual(
        Box((0.94, 0.86, 0.006)),
        origin=Origin(xyz=(0.430, 0.0, -0.003)),
        material=glass,
        name="windshield_patch",
    )
    mount.visual(
        Box((0.255, 0.88, 0.020)),
        origin=Origin(xyz=(-0.075, 0.0, -0.010)),
        material=satin_black,
        name="cowl_panel",
    )
    mount.visual(
        Cylinder(radius=0.074, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=rubber_black,
        name="rubber_gasket",
    )
    mount.visual(
        Cylinder(radius=0.048, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=dark_metal,
        name="pivot_pedestal",
    )
    mount.visual(
        Cylinder(radius=0.026, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=brushed_steel,
        name="splined_spindle",
    )

    arm = model.part("arm")
    arm.visual(
        mesh_from_geometry(_hub_cap(), "hub_cap"),
        material=satin_black,
        name="hub_cap",
    )
    arm.visual(
        mesh_from_geometry(
            CylinderGeometry(0.018, 0.010, radial_segments=6, closed=True),
            "retaining_nut",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=zinc,
        name="retaining_nut",
    )
    arm.visual(
        mesh_from_geometry(_lightened_arm(), "tapered_arm"),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=satin_black,
        name="tapered_arm",
    )
    arm.visual(
        Box((0.420, 0.006, 0.007)),
        origin=Origin(xyz=(0.315, -0.015, 0.022)),
        material=dark_metal,
        name="pressed_rib_0",
    )
    arm.visual(
        Box((0.420, 0.006, 0.007)),
        origin=Origin(xyz=(0.315, 0.015, 0.022)),
        material=dark_metal,
        name="pressed_rib_1",
    )
    arm.visual(
        Box((0.022, 0.026, 0.014)),
        origin=Origin(xyz=(0.150, 0.031, 0.017)),
        material=dark_metal,
        name="spring_anchor_0",
    )
    arm.visual(
        Box((0.022, 0.026, 0.014)),
        origin=Origin(xyz=(0.365, 0.031, 0.017)),
        material=dark_metal,
        name="spring_anchor_1",
    )
    arm.visual(
        mesh_from_geometry(_coil_spring(), "return_spring"),
        material=brushed_steel,
        name="return_spring",
    )
    arm.visual(
        Cylinder(radius=0.008, length=0.006),
        origin=Origin(xyz=(0.145, 0.0, 0.0205)),
        material=zinc,
        name="rivet_0",
    )
    arm.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(xyz=(0.500, 0.0, 0.0205)),
        material=zinc,
        name="rivet_1",
    )
    arm.visual(
        Box((0.023, 0.092, 0.014)),
        origin=Origin(xyz=(0.546, 0.0, 0.014)),
        material=satin_black,
        name="fork_crossbar",
    )
    arm.visual(
        Box((0.086, 0.014, 0.016)),
        origin=Origin(xyz=(0.588, -0.040, 0.014)),
        material=satin_black,
        name="fork_prong_0",
    )
    arm.visual(
        Box((0.086, 0.014, 0.016)),
        origin=Origin(xyz=(0.588, 0.040, 0.014)),
        material=satin_black,
        name="fork_prong_1",
    )
    arm.visual(
        Cylinder(radius=0.006, length=0.086),
        origin=Origin(xyz=(0.622, 0.0, 0.014), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="tip_pin",
    )

    blade = model.part("blade")
    blade.visual(
        Box((0.058, 0.072, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_metal,
        name="adapter_block",
    )
    blade.visual(
        Box((0.018, 0.036, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=dark_metal,
        name="bridge_stem",
    )
    blade.visual(
        Box((0.026, 0.034, 0.038)),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=dark_metal,
        name="center_strut",
    )
    blade.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.0, -0.245, 0.006),
                    (0.0, -0.120, 0.022),
                    (0.0, 0.000, 0.034),
                    (0.0, 0.120, 0.022),
                    (0.0, 0.245, 0.006),
                ],
                radius=0.0055,
                samples_per_segment=16,
                radial_segments=14,
                cap_ends=True,
            ),
            "primary_bridge",
        ),
        material=satin_black,
        name="primary_bridge",
    )
    blade.visual(
        Box((0.040, 0.740, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.042)),
        material=satin_black,
        name="steel_spine",
    )
    blade.visual(
        Box((0.006, 0.720, 0.008)),
        origin=Origin(xyz=(-0.020, 0.0, -0.039)),
        material=brushed_steel,
        name="rail_0",
    )
    blade.visual(
        Box((0.006, 0.720, 0.008)),
        origin=Origin(xyz=(0.020, 0.0, -0.039)),
        material=brushed_steel,
        name="rail_1",
    )
    blade.visual(
        mesh_from_geometry(_rubber_wedge(0.740, 0.030, 0.026), "rubber_wedge"),
        origin=Origin(xyz=(0.0, 0.0, -0.038)),
        material=rubber_black,
        name="rubber_wedge",
    )
    blade.visual(
        Box((0.040, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.375, -0.030)),
        material=rubber_black,
        name="end_cap_0",
    )
    blade.visual(
        Box((0.040, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.375, -0.030)),
        material=rubber_black,
        name="end_cap_1",
    )
    blade.visual(
        Cylinder(radius=0.010, length=0.044),
        origin=Origin(xyz=(0.0, -0.185, -0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=zinc,
        name="yoke_pin_0",
    )
    blade.visual(
        Box((0.006, 0.018, 0.018)),
        origin=Origin(xyz=(-0.0235, -0.185, -0.034)),
        material=dark_metal,
        name="pin_lug_0a",
    )
    blade.visual(
        Box((0.006, 0.018, 0.018)),
        origin=Origin(xyz=(0.0235, -0.185, -0.034)),
        material=dark_metal,
        name="pin_lug_0b",
    )
    blade.visual(
        Cylinder(radius=0.010, length=0.044),
        origin=Origin(xyz=(0.0, 0.185, -0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=zinc,
        name="yoke_pin_1",
    )
    blade.visual(
        Box((0.006, 0.018, 0.018)),
        origin=Origin(xyz=(-0.0235, 0.185, -0.034)),
        material=dark_metal,
        name="pin_lug_1a",
    )
    blade.visual(
        Box((0.006, 0.018, 0.018)),
        origin=Origin(xyz=(0.0235, 0.185, -0.034)),
        material=dark_metal,
        name="pin_lug_1b",
    )

    for idx, y_sign in enumerate((-1.0, 1.0)):
        yoke = model.part(f"yoke_{idx}")
        yoke.visual(
            Cylinder(radius=0.014, length=0.034),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_metal,
            name="yoke_hub",
        )
        yoke.visual(
            Box((0.058, 0.205, 0.009)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=satin_black,
            name="rocker_bridge",
        )
        yoke.visual(
            Box((0.006, 0.015, 0.036)),
            origin=Origin(xyz=(-0.025, -0.084, -0.017)),
            material=satin_black,
            name="claw_0",
        )
        yoke.visual(
            Box((0.006, 0.015, 0.036)),
            origin=Origin(xyz=(0.025, -0.084, -0.017)),
            material=satin_black,
            name="claw_1",
        )
        yoke.visual(
            Box((0.006, 0.015, 0.036)),
            origin=Origin(xyz=(-0.025, 0.084, -0.017)),
            material=satin_black,
            name="claw_2",
        )
        yoke.visual(
            Box((0.006, 0.015, 0.036)),
            origin=Origin(xyz=(0.025, 0.084, -0.017)),
            material=satin_black,
            name="claw_3",
        )
        yoke.visual(
            Box((0.010, 0.012, 0.006)),
            origin=Origin(xyz=(0.032, -0.084, -0.037)),
            material=satin_black,
            name="clip_0",
        )
        yoke.visual(
            Box((0.010, 0.012, 0.006)),
            origin=Origin(xyz=(0.032, 0.084, -0.037)),
            material=satin_black,
            name="clip_1",
        )

        model.articulation(
            f"yoke_pivot_{idx}",
            ArticulationType.REVOLUTE,
            parent=blade,
            child=yoke,
            origin=Origin(xyz=(0.0, y_sign * 0.185, -0.018)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=0.8, lower=-0.22, upper=0.22),
        )

    model.articulation(
        "base_pivot",
        ArticulationType.REVOLUTE,
        parent=mount,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=2.0, lower=-1.05, upper=1.05),
    )
    model.articulation(
        "tip_pivot",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=blade,
        origin=Origin(xyz=(0.622, 0.0, 0.014)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.4, lower=-0.35, upper=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mount = object_model.get_part("mount")
    arm = object_model.get_part("arm")
    blade = object_model.get_part("blade")
    yoke_0 = object_model.get_part("yoke_0")
    yoke_1 = object_model.get_part("yoke_1")
    base_pivot = object_model.get_articulation("base_pivot")

    ctx.allow_overlap(
        arm,
        blade,
        elem_a="tip_pin",
        elem_b="adapter_block",
        reason="The blade adapter is intentionally captured around the arm's cross pin.",
    )
    ctx.expect_overlap(
        arm,
        blade,
        axes="xyz",
        elem_a="tip_pin",
        elem_b="adapter_block",
        min_overlap=0.006,
        name="tip pin passes through adapter block",
    )

    for idx, yoke in enumerate((yoke_0, yoke_1)):
        ctx.allow_overlap(
            blade,
            yoke,
            elem_a=f"yoke_pin_{idx}",
            elem_b="yoke_hub",
            reason="The rocker yoke hub is modeled as a captured bushing around its pin.",
        )
        ctx.expect_overlap(
            blade,
            yoke,
            axes="xyz",
            elem_a=f"yoke_pin_{idx}",
            elem_b="yoke_hub",
            min_overlap=0.006,
            name=f"yoke {idx} retained on pivot pin",
        )
        ctx.allow_overlap(
            blade,
            yoke,
            elem_a=f"yoke_pin_{idx}",
            elem_b="rocker_bridge",
            reason="The central rocker bridge is modeled with a solid pivot boss pierced by the yoke pin.",
        )
        ctx.expect_overlap(
            blade,
            yoke,
            axes="xyz",
            elem_a=f"yoke_pin_{idx}",
            elem_b="rocker_bridge",
            min_overlap=0.006,
            name=f"yoke {idx} bridge pierced by pivot pin",
        )

    ctx.expect_gap(
        blade,
        mount,
        axis="z",
        positive_elem="rubber_wedge",
        negative_elem="windshield_patch",
        max_gap=0.0015,
        max_penetration=0.0005,
        name="rubber wiping edge rests on windshield",
    )
    ctx.expect_contact(
        arm,
        mount,
        elem_a="hub_cap",
        elem_b="pivot_pedestal",
        contact_tol=0.001,
        name="arm hub seated on pivot pedestal",
    )

    rest_pos = ctx.part_world_position(blade)
    with ctx.pose({base_pivot: 0.75}):
        swept_pos = ctx.part_world_position(blade)
    ctx.check(
        "base pivot sweeps blade across windshield",
        rest_pos is not None
        and swept_pos is not None
        and swept_pos[1] > rest_pos[1] + 0.35
        and abs(swept_pos[2] - rest_pos[2]) < 0.005,
        details=f"rest={rest_pos}, swept={swept_pos}",
    )

    return ctx.report()


object_model = build_object_model()
