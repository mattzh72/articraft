from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _bin_shell_geometry() -> object:
    """Open tapered wheelie-bin tub with a real cavity and raised rim."""

    outer = (
        cq.Workplane("XY")
        .workplane(offset=0.18)
        .rect(0.46, 0.50)
        .workplane(offset=0.86)
        .rect(0.70, 0.64)
        .loft(combine=True)
    )
    inner_void = (
        cq.Workplane("XY")
        .workplane(offset=0.225)
        .rect(0.38, 0.42)
        .workplane(offset=0.92)
        .rect(0.60, 0.54)
        .loft(combine=True)
    )
    rim = (
        cq.Workplane("XY")
        .workplane(offset=1.035)
        .rect(0.74, 0.67)
        .rect(0.60, 0.53)
        .extrude(0.035)
    )
    return outer.union(rim).cut(inner_void)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_calibration_wheelie_bin")

    bin_green = model.material("bin_green", rgba=(0.05, 0.24, 0.17, 1.0))
    datum_grey = model.material("datum_grey", rgba=(0.62, 0.68, 0.66, 1.0))
    black = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    dark = model.material("dark_hardware", rgba=(0.08, 0.09, 0.09, 1.0))
    white = model.material("white_index", rgba=(0.88, 0.92, 0.86, 1.0))
    blue = model.material("blue_adjuster", rgba=(0.05, 0.18, 0.8, 1.0))

    shell = model.part("bin_shell")
    shell.visual(
        mesh_from_cadquery(_bin_shell_geometry(), "open_tapered_bin_shell", tolerance=0.002),
        material=bin_green,
        name="tub_shell",
    )

    # Rear axle, wheel cradles, and push handle are molded into the root shell.
    shell.visual(
        Cylinder(radius=0.018, length=0.84),
        origin=Origin(xyz=(0.315, 0.0, 0.125), rpy=(pi / 2, 0.0, 0.0)),
        material=dark,
        name="rear_axle",
    )
    for i, y in enumerate((-0.255, 0.255)):
        shell.visual(
            Box((0.16, 0.100, 0.145)),
            origin=Origin(xyz=(0.305, y, 0.170)),
            material=bin_green,
            name=f"axle_cradle_{i}",
        )
    shell.visual(
        Cylinder(radius=0.017, length=0.61),
        origin=Origin(xyz=(0.395, 0.0, 0.965), rpy=(pi / 2, 0.0, 0.0)),
        material=bin_green,
        name="rear_handle_bar",
    )
    for i, y in enumerate((-0.255, 0.255)):
        shell.visual(
            Box((0.065, 0.042, 0.18)),
            origin=Origin(xyz=(0.365, y, 0.91)),
            material=bin_green,
            name=f"handle_stanchion_{i}",
        )

    # Explicit hinge hardware: a full-width pin supported by rear yokes.
    shell.visual(
        Cylinder(radius=0.012, length=0.88),
        origin=Origin(xyz=(0.365, 0.0, 1.105), rpy=(pi / 2, 0.0, 0.0)),
        material=dark,
        name="rear_hinge_pin",
    )
    for i, y in enumerate((-0.395, 0.395)):
        shell.visual(
            Box((0.075, 0.060, 0.095)),
            origin=Origin(xyz=(0.365, y, 1.080)),
            material=bin_green,
            name=f"hinge_yoke_{i}",
        )
    for i, y in enumerate((-0.360, 0.360)):
        shell.visual(
            Box((0.075, 0.078, 0.055)),
            origin=Origin(xyz=(0.365, y, 1.015)),
            material=bin_green,
            name=f"hinge_bridge_{i}",
        )

    # Datum-friendly pads and controlled gap reference strips around the rim.
    shell.visual(
        Box((0.010, 0.60, 0.014)),
        origin=Origin(xyz=(-0.345, 0.0, 1.063)),
        material=black,
        name="front_gap_bar",
    )
    for i, y in enumerate((-0.295, 0.295)):
        shell.visual(
            Box((0.620, 0.012, 0.016)),
            origin=Origin(xyz=(0.025, y, 1.060)),
            material=black,
            name=f"side_gap_bar_{i}",
        )
    for i, y in enumerate((-0.18, 0.0, 0.18)):
        shell.visual(
            Box((0.040, 0.105, 0.008)),
            origin=Origin(xyz=(-0.330, y, 1.074)),
            material=datum_grey,
            name=f"front_datum_pad_{i}",
        )
    shell.visual(
        Box((0.58, 0.030, 0.007)),
        origin=Origin(xyz=(0.155, -0.295, 1.071)),
        material=datum_grey,
        name="side_datum_rail",
    )
    shell.visual(
        Box((0.58, 0.030, 0.012)),
        origin=Origin(xyz=(0.155, 0.295, 1.067)),
        material=datum_grey,
        name="side_datum_rail_1",
    )

    # Rear hinge index scale: short marks on the shell, with a pointer on the lid.
    shell.visual(
        Box((0.155, 0.018, 0.010)),
        origin=Origin(xyz=(0.290, -0.332, 1.020)),
        material=bin_green,
        name="hinge_index_backing",
    )
    for i, x in enumerate((0.230, 0.260, 0.290, 0.320, 0.350)):
        shell.visual(
            Box((0.006, 0.028 if i % 2 == 0 else 0.018, 0.004)),
            origin=Origin(xyz=(x, -0.337, 1.033)),
            material=white,
            name=f"hinge_index_{i}",
        )

    # Threaded adjustment bosses which carry rotating gap stop screws.
    for i, y in enumerate((-0.225, 0.225)):
        shell.visual(
            Box((0.180, 0.060, 0.050)),
            origin=Origin(xyz=(-0.405, y, 0.965)),
            material=bin_green,
            name=f"stop_boss_{i}",
        )

    lid = model.part("lid")
    lid.visual(
        Box((0.728, 0.680, 0.035)),
        origin=Origin(xyz=(-0.386, 0.0, -0.010)),
        material=bin_green,
        name="lid_panel",
    )
    lid.visual(
        Box((0.036, 0.690, 0.060)),
        origin=Origin(xyz=(-0.766, 0.0, -0.036)),
        material=bin_green,
        name="front_lip",
    )
    for i, y in enumerate((-0.180, 0.180)):
        lid.visual(
            Box((0.720, 0.024, 0.050)),
            origin=Origin(xyz=(-0.385, y, 0.020)),
            material=bin_green,
            name=f"side_lip_{i}",
        )
    lid.visual(
        Cylinder(radius=0.026, length=0.43),
        origin=Origin(xyz=(0.0, 0.0, 0.000), rpy=(pi / 2, 0.0, 0.0)),
        material=bin_green,
        name="lid_barrel",
    )
    lid.visual(
        Box((0.200, 0.150, 0.007)),
        origin=Origin(xyz=(-0.390, 0.0, 0.010)),
        material=datum_grey,
        name="top_datum_plate",
    )
    for i, y in enumerate((-0.245, 0.245)):
        lid.visual(
            Box((0.390, 0.020, 0.007)),
            origin=Origin(xyz=(-0.360, y, 0.010)),
            material=datum_grey,
            name=f"parallel_datum_rail_{i}",
        )
    lid.visual(
        Box((0.030, 0.008, 0.014)),
        origin=Origin(xyz=(-0.040, -0.337, 0.006)),
        material=white,
        name="angle_pointer",
    )
    for i, x in enumerate((-0.55, -0.48, -0.41, -0.34, -0.27)):
        lid.visual(
            Box((0.006, 0.050, 0.006)),
            origin=Origin(xyz=(x, 0.315, 0.010)),
            material=white,
            name=f"lid_index_{i}",
        )

    model.articulation(
        "shell_to_lid",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=lid,
        origin=Origin(xyz=(0.365, 0.0, 1.105)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.2, lower=0.0, upper=1.92),
        motion_properties=MotionProperties(damping=0.08, friction=0.02),
    )

    wheel_mesh = WheelGeometry(
        0.092,
        0.070,
        rim=WheelRim(inner_radius=0.054, flange_height=0.006, flange_thickness=0.004),
        hub=WheelHub(
            radius=0.034,
            width=0.054,
            cap_style="flat",
            bolt_pattern=BoltPattern(count=6, circle_diameter=0.048, hole_diameter=0.004),
        ),
        face=WheelFace(dish_depth=0.005, front_inset=0.002, rear_inset=0.002),
        spokes=WheelSpokes(style="straight", count=6, thickness=0.005, window_radius=0.010),
        bore=WheelBore(style="round", diameter=0.025),
    )
    tire_mesh = TireGeometry(
        0.122,
        0.074,
        inner_radius=0.084,
        tread=TireTread(style="block", depth=0.005, count=20, land_ratio=0.55),
        sidewall=TireSidewall(style="square", bulge=0.02),
        shoulder=TireShoulder(width=0.006, radius=0.003),
    )

    for i, y in enumerate((-0.355, 0.355)):
        wheel = model.part(f"wheel_{i}")
        wheel.visual(
            mesh_from_geometry(tire_mesh, f"wheel_{i}_tire"),
            origin=Origin(rpy=(0.0, 0.0, pi / 2)),
            material=black,
            name="tire",
        )
        wheel.visual(
            mesh_from_geometry(wheel_mesh, f"wheel_{i}_hub"),
            origin=Origin(rpy=(0.0, 0.0, pi / 2)),
            material=datum_grey,
            name="hub",
        )
        model.articulation(
            f"shell_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=shell,
            child=wheel,
            origin=Origin(xyz=(0.315, y, 0.125)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=5.0, velocity=20.0),
            motion_properties=MotionProperties(damping=0.01, friction=0.015),
        )

    for i, y in enumerate((-0.225, 0.225)):
        screw = model.part(f"stop_screw_{i}")
        screw.visual(
            Cylinder(radius=0.0065, length=0.090),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=dark,
            name="screw_shaft",
        )
        screw.visual(
            Cylinder(radius=0.027, length=0.016),
            origin=Origin(xyz=(0.0, 0.0, 0.046)),
            material=blue,
            name="thumbwheel",
        )
        screw.visual(
            Box((0.045, 0.004, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, 0.055)),
            material=white,
            name="witness_line",
        )
        model.articulation(
            f"shell_to_stop_screw_{i}",
            ArticulationType.CONTINUOUS,
            parent=shell,
            child=screw,
            origin=Origin(xyz=(-0.435, y, 0.965)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=1.0, velocity=4.0),
            motion_properties=MotionProperties(damping=0.02, friction=0.03),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shell = object_model.get_part("bin_shell")
    lid = object_model.get_part("lid")
    lid_joint = object_model.get_articulation("shell_to_lid")

    ctx.allow_overlap(
        shell,
        lid,
        elem_a="rear_hinge_pin",
        elem_b="lid_barrel",
        reason="The lid barrel is intentionally captured around the full-width hinge pin.",
    )
    ctx.expect_overlap(
        lid,
        shell,
        axes="y",
        elem_a="lid_barrel",
        elem_b="rear_hinge_pin",
        min_overlap=0.30,
        name="lid barrel spans the hinge pin",
    )
    ctx.expect_overlap(
        lid,
        shell,
        axes="xz",
        elem_a="lid_barrel",
        elem_b="rear_hinge_pin",
        min_overlap=0.015,
        name="lid barrel is coaxial with the hinge pin",
    )
    ctx.expect_gap(
        lid,
        shell,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="front_gap_bar",
        min_gap=0.006,
        max_gap=0.020,
        name="closed lid keeps a controlled front datum gap",
    )
    ctx.expect_overlap(
        lid,
        shell,
        axes="xy",
        elem_a="lid_panel",
        elem_b="front_gap_bar",
        min_overlap=0.009,
        name="lid covers the front gap datum bar",
    )

    closed_lip_aabb = ctx.part_element_world_aabb(lid, elem="front_lip")
    with ctx.pose({lid_joint: 1.20}):
        open_lip_aabb = ctx.part_element_world_aabb(lid, elem="front_lip")
    closed_lip_z = closed_lip_aabb[0][2] if closed_lip_aabb is not None else None
    open_lip_z = open_lip_aabb[0][2] if open_lip_aabb is not None else None
    ctx.check(
        "lid swing raises the front lip",
        closed_lip_z is not None and open_lip_z is not None and open_lip_z > closed_lip_z + 0.25,
        details=f"closed_z={closed_lip_z}, open_z={open_lip_z}",
    )

    for i in range(2):
        wheel = object_model.get_part(f"wheel_{i}")
        ctx.allow_overlap(
            shell,
            wheel,
            elem_a="rear_axle",
            elem_b="hub",
            reason="The rolling wheel hub is intentionally captured by the rear axle.",
        )
        ctx.expect_overlap(
            wheel,
            shell,
            axes="y",
            elem_a="hub",
            elem_b="rear_axle",
            min_overlap=0.060,
            name=f"wheel {i} hub has retained axle width",
        )
        ctx.expect_overlap(
            wheel,
            shell,
            axes="xz",
            elem_a="hub",
            elem_b="rear_axle",
            min_overlap=0.015,
            name=f"wheel {i} hub is centered on axle",
        )

        screw = object_model.get_part(f"stop_screw_{i}")
        ctx.allow_overlap(
            shell,
            screw,
            elem_a=f"stop_boss_{i}",
            elem_b="screw_shaft",
            reason="The threaded gap stop screw passes through its molded adjustment boss.",
        )
        ctx.expect_overlap(
            screw,
            shell,
            axes="xyz",
            elem_a="screw_shaft",
            elem_b=f"stop_boss_{i}",
            min_overlap=0.010,
            name=f"stop screw {i} is retained in its boss",
        )

    wheel_joints = [object_model.get_articulation(f"shell_to_wheel_{i}") for i in range(2)]
    ctx.check(
        "wheel joints roll on rear axle",
        all(j.axis == (0.0, 1.0, 0.0) and j.articulation_type == ArticulationType.CONTINUOUS for j in wheel_joints),
        details=f"wheel joint axes={[j.axis for j in wheel_joints]}",
    )
    ctx.check(
        "lid hinge has repeatable bounded swing",
        lid_joint.axis == (0.0, 1.0, 0.0)
        and lid_joint.motion_limits is not None
        and lid_joint.motion_limits.lower == 0.0
        and lid_joint.motion_limits.upper is not None
        and lid_joint.motion_limits.upper > 1.5,
        details=f"axis={lid_joint.axis}, limits={lid_joint.motion_limits}",
    )
    return ctx.report()


object_model = build_object_model()
