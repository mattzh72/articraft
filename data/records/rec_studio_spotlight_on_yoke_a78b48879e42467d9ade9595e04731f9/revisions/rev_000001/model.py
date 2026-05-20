from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_spotlight_tripod")

    black = model.material("satin_black", rgba=(0.005, 0.005, 0.004, 1.0))
    dark = model.material("dark_hardware", rgba=(0.02, 0.018, 0.015, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.01, 0.01, 0.009, 1.0))
    reflector = model.material("brushed_reflector", rgba=(0.78, 0.73, 0.62, 1.0))
    warm_glass = model.material("warm_bulb", rgba=(1.0, 0.76, 0.34, 0.72))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.074, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.600)),
        material=dark,
        name="tripod_crown",
    )
    stand.visual(
        Cylinder(radius=0.025, length=0.420),
        origin=Origin(xyz=(0.0, 0.0, 0.810)),
        material=black,
        name="lower_mast",
    )
    stand.visual(
        Cylinder(radius=0.017, length=0.170),
        origin=Origin(xyz=(0.0, 0.0, 1.090)),
        material=black,
        name="upper_mast",
    )
    stand.visual(
        Cylinder(radius=0.039, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 1.005)),
        material=dark,
        name="mast_collar",
    )
    stand.visual(
        Cylinder(radius=0.052, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 1.180)),
        material=dark,
        name="pan_socket",
    )

    leg_angles = [math.radians(90.0), math.radians(210.0), math.radians(330.0)]
    for index, angle in enumerate(leg_angles):
        radial = (math.cos(angle), math.sin(angle))
        tangent_yaw = angle + math.pi / 2.0
        stand.visual(
            Box((0.095, 0.026, 0.032)),
            origin=Origin(
                xyz=(0.083 * radial[0], 0.083 * radial[1], 0.602),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark,
            name=f"leg_hinge_{index}",
        )

        leg = model.part(f"leg_{index}")
        leg_tube = wire_from_points(
            [
                (0.0, 0.0, 0.0),
                (0.13 * radial[0], 0.13 * radial[1], -0.19),
                (0.47 * radial[0], 0.47 * radial[1], -0.58),
                (0.57 * radial[0], 0.57 * radial[1], -0.60),
            ],
            radius=0.012,
            radial_segments=16,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.035,
        )
        leg.visual(
            mesh_from_geometry(leg_tube, f"leg_{index}_tube"),
            material=black,
            name="leg_tube",
        )
        leg.visual(
            Cylinder(radius=0.017, length=0.052),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, tangent_yaw)),
            material=dark,
            name="hinge_barrel",
        )
        leg.visual(
            Box((0.12, 0.035, 0.016)),
            origin=Origin(
                xyz=(0.59 * radial[0], 0.59 * radial[1], -0.605),
                rpy=(0.0, 0.0, angle),
            ),
            material=rubber,
            name="foot_pad",
        )
        model.articulation(
            f"stand_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=stand,
            child=leg,
            origin=Origin(xyz=(0.103 * radial[0], 0.103 * radial[1], 0.602)),
            axis=(-math.sin(angle), math.cos(angle), 0.0),
            motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=-0.45, upper=0.65),
        )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.047, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark,
        name="pan_boss",
    )
    yoke.visual(
        Box((0.060, 0.390, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=black,
        name="yoke_crossbar",
    )
    for index, y in enumerate((-0.182, 0.182)):
        yoke.visual(
            Box((0.034, 0.026, 0.245)),
            origin=Origin(xyz=(0.0, y, 0.165)),
            material=black,
            name=f"yoke_arm_{index}",
        )
        yoke.visual(
            Cylinder(radius=0.035, length=0.028),
            origin=Origin(xyz=(0.0, y, 0.245), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark,
            name=f"tilt_cheek_{index}",
        )
    model.articulation(
        "stand_to_yoke",
        ArticulationType.CONTINUOUS,
        parent=stand,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 1.200)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0),
    )

    can = model.part("can")
    can_shell = LatheGeometry(
        [
            (0.104, -0.190),
            (0.122, -0.130),
            (0.134, 0.215),
            (0.119, 0.235),
            (0.106, 0.207),
            (0.071, -0.145),
            (0.000, -0.154),
            (0.000, -0.190),
        ],
        segments=72,
        closed=True,
    )
    can.visual(
        mesh_from_geometry(can_shell, "lamp_can_shell"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="can_shell",
    )
    reflector_mesh = LatheGeometry(
        [
            (0.045, -0.130),
            (0.108, 0.180),
            (0.103, 0.194),
            (0.038, -0.116),
        ],
        segments=64,
        closed=True,
    )
    can.visual(
        mesh_from_geometry(reflector_mesh, "inner_reflector"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=reflector,
        name="reflector",
    )
    can.visual(
        Cylinder(radius=0.018, length=0.115),
        origin=Origin(xyz=(-0.103, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="bulb_socket",
    )
    can.visual(
        Sphere(radius=0.036),
        origin=Origin(xyz=(-0.030, 0.0, 0.0)),
        material=warm_glass,
        name="bulb",
    )
    for index, y in enumerate((-0.149, 0.149)):
        can.visual(
            Cylinder(radius=0.028, length=0.070),
            origin=Origin(xyz=(0.0, y * 0.93, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark,
            name=f"tilt_trunnion_{index}",
        )
    can.visual(
        Cylinder(radius=0.022, length=0.020),
        origin=Origin(xyz=(-0.085, -0.123, 0.020), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="focus_boss",
    )
    for suffix, z in (("top", 0.132), ("bottom", -0.132)):
        can.visual(
            Box((0.035, 0.255, 0.052)),
            origin=Origin(xyz=(0.242, 0.0, z)),
            material=dark,
            name=f"{suffix}_hinge_mount",
        )
    for suffix, y in (("side_0", 0.132), ("side_1", -0.132)):
        can.visual(
            Box((0.035, 0.052, 0.255)),
            origin=Origin(xyz=(0.242, y, 0.0)),
            material=dark,
            name=f"{suffix}_hinge_mount",
        )
    model.articulation(
        "yoke_to_can",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=can,
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=-0.75, upper=0.90),
    )

    knob = model.part("focus_knob")
    knob.visual(
        Cylinder(radius=0.007, length=0.030),
        origin=Origin(xyz=(0.0, -0.015, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="shaft",
    )
    focus_knob_mesh = KnobGeometry(
        0.056,
        0.026,
        body_style="faceted",
        top_diameter=0.047,
        edge_radius=0.001,
        grip=KnobGrip(style="ribbed", count=18, depth=0.0020, width=0.0020),
        indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
    )
    knob.visual(
        mesh_from_geometry(focus_knob_mesh, "focus_knob_cap"),
        origin=Origin(xyz=(0.0, -0.040, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="knob_cap",
    )
    model.articulation(
        "can_to_focus_knob",
        ArticulationType.CONTINUOUS,
        parent=can,
        child=knob,
        origin=Origin(xyz=(-0.085, -0.133, 0.020)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )

    front_x = 0.252
    front_r = 0.150
    leaf_specs = [
        ("top_leaf", (front_x, 0.0, front_r), (0.0, 1.0, 0.0), (0.060, 0.0, 0.090), (0.0, 0.55, 0.0), Box((0.008, 0.255, 0.190))),
        ("bottom_leaf", (front_x, 0.0, -front_r), (0.0, 1.0, 0.0), (0.060, 0.0, -0.090), (0.0, -0.55, 0.0), Box((0.008, 0.255, 0.190))),
        ("side_leaf_0", (front_x, front_r, 0.0), (0.0, 0.0, 1.0), (0.060, 0.090, 0.0), (0.0, 0.0, -0.55), Box((0.008, 0.190, 0.255))),
        ("side_leaf_1", (front_x, -front_r, 0.0), (0.0, 0.0, 1.0), (0.060, -0.090, 0.0), (0.0, 0.0, 0.55), Box((0.008, 0.190, 0.255))),
    ]
    for name, joint_xyz, axis, panel_xyz, panel_rpy, panel_box in leaf_specs:
        leaf = model.part(name)
        leaf.visual(
            Cylinder(radius=0.0075, length=0.255 if axis[1] else 0.190),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0) if axis[1] else Origin().rpy),
            material=dark,
            name="hinge_barrel",
        )
        if name in ("top_leaf", "bottom_leaf"):
            sign = 1.0 if name == "top_leaf" else -1.0
            leaf.visual(
                Box((0.050, 0.240, 0.034)),
                origin=Origin(xyz=(0.024, 0.0, 0.020 * sign)),
                material=dark,
                name="hinge_leaf",
            )
        else:
            sign = 1.0 if name == "side_leaf_0" else -1.0
            leaf.visual(
                Box((0.050, 0.034, 0.240)),
                origin=Origin(xyz=(0.024, 0.020 * sign, 0.0)),
                material=dark,
                name="hinge_leaf",
            )
        leaf.visual(
            panel_box,
            origin=Origin(xyz=panel_xyz, rpy=panel_rpy),
            material=black,
            name="leaf_panel",
        )
        model.articulation(
            f"can_to_{name}",
            ArticulationType.REVOLUTE,
            parent=can,
            child=leaf,
            origin=Origin(xyz=joint_xyz),
            axis=axis,
            motion_limits=MotionLimits(effort=1.5, velocity=3.0, lower=-1.10, upper=1.25),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    required_joints = [
        "stand_to_leg_0",
        "stand_to_leg_1",
        "stand_to_leg_2",
        "stand_to_yoke",
        "yoke_to_can",
        "can_to_focus_knob",
        "can_to_top_leaf",
        "can_to_bottom_leaf",
        "can_to_side_leaf_0",
        "can_to_side_leaf_1",
    ]
    ctx.check(
        "all spotlight mechanisms are articulated",
        all(object_model.get_articulation(name) is not None for name in required_joints),
        details=f"required={required_joints}",
    )

    for index in range(3):
        leg = f"leg_{index}"
        ctx.allow_overlap(
            leg,
            "stand",
            elem_a="hinge_barrel",
            elem_b=f"leg_hinge_{index}",
            reason="The tripod leg hinge barrel is intentionally captured inside the crown hinge lug.",
        )
        ctx.expect_overlap(
            leg,
            "stand",
            elem_a="hinge_barrel",
            elem_b=f"leg_hinge_{index}",
            axes="xyz",
            min_overlap=0.008,
            name=f"leg {index} hinge barrel is retained by the crown lug",
        )
        ctx.allow_overlap(
            leg,
            "stand",
            elem_a="leg_tube",
            elem_b=f"leg_hinge_{index}",
            reason="The upper tripod tube intentionally enters the hinge lug at the crown pivot.",
        )
        ctx.expect_overlap(
            leg,
            "stand",
            elem_a="leg_tube",
            elem_b=f"leg_hinge_{index}",
            axes="xyz",
            min_overlap=0.008,
            name=f"leg {index} upper tube enters the crown hinge lug",
        )

    for leaf, mount in [
        ("top_leaf", "top_hinge_mount"),
        ("bottom_leaf", "bottom_hinge_mount"),
        ("side_leaf_0", "side_0_hinge_mount"),
        ("side_leaf_1", "side_1_hinge_mount"),
    ]:
        ctx.allow_overlap(
            leaf,
            "can",
            elem_a="hinge_barrel",
            elem_b=mount,
            reason="The barndoor hinge barrel is locally captured by its front can hinge mount.",
        )
        ctx.expect_overlap(
            leaf,
            "can",
            elem_a="hinge_barrel",
            elem_b=mount,
            axes="xyz",
            min_overlap=0.008,
            name=f"{leaf} hinge barrel is retained by the can mount",
        )
        ctx.allow_overlap(
            leaf,
            "can",
            elem_a="hinge_leaf",
            elem_b=mount,
            reason="The barndoor hinge leaf is intentionally nested in the can-side hinge bracket.",
        )
        ctx.expect_overlap(
            leaf,
            "can",
            elem_a="hinge_leaf",
            elem_b=mount,
            axes="xyz",
            min_overlap=0.003,
            name=f"{leaf} hinge leaf is nested in the can mount",
        )

    for index in range(2):
        ctx.allow_overlap(
            "can",
            "yoke",
            elem_a=f"tilt_trunnion_{index}",
            elem_b=f"tilt_cheek_{index}",
            reason="The lamp can tilt trunnion is intentionally seated in the yoke cheek bearing.",
        )
        ctx.expect_overlap(
            "can",
            "yoke",
            elem_a=f"tilt_trunnion_{index}",
            elem_b=f"tilt_cheek_{index}",
            axes="xyz",
            min_overlap=0.003,
            name=f"tilt trunnion {index} is seated in the yoke cheek",
        )

    return ctx.report()


object_model = build_object_model()
