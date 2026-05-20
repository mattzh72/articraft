from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


PAN_STATIONS = (
    ((1.70, 0.00, 7.90), -pi / 2.0),
    ((-1.70, 0.00, 7.90), pi / 2.0),
    ((0.00, 1.70, 7.90), 0.0),
    ((0.00, -1.70, 7.90), pi),
)

TILT_ORIGIN = (0.0, 0.36, -0.48)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stadium_four_head_lighting_mast")

    concrete = model.material("weathered_concrete", rgba=(0.58, 0.57, 0.53, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.62, 0.65, 0.64, 1.0))
    dark_steel = model.material("dark_powder_coated_steel", rgba=(0.055, 0.060, 0.065, 1.0))
    glass = model.material("cool_glass_lens", rgba=(0.72, 0.90, 1.00, 0.58))
    led = model.material("warm_led_diode", rgba=(1.00, 0.86, 0.38, 1.0))
    rubber = model.material("black_cable_gland", rgba=(0.015, 0.015, 0.014, 1.0))

    mast = model.part("mast")
    mast.visual(
        Box((2.40, 2.40, 0.35)),
        origin=Origin(xyz=(0.0, 0.0, 0.175)),
        material=concrete,
        name="concrete_base",
    )
    mast.visual(
        Box((0.74, 0.74, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.3725)),
        material=galvanized,
        name="base_plate",
    )
    mast.visual(
        Cylinder(radius=0.12, length=7.655),
        origin=Origin(xyz=(0.0, 0.0, 4.2225)),
        material=galvanized,
        name="round_pole",
    )
    mast.visual(
        Cylinder(radius=0.16, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 8.02)),
        material=galvanized,
        name="top_hub",
    )
    mast.visual(
        Cylinder(radius=0.065, length=3.58),
        origin=Origin(xyz=(0.0, 0.0, 8.05), rpy=(0.0, pi / 2.0, 0.0)),
        material=galvanized,
        name="cross_arm_x",
    )
    mast.visual(
        Cylinder(radius=0.065, length=3.58),
        origin=Origin(xyz=(0.0, 0.0, 8.05), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="cross_arm_y",
    )

    # Welded collars at the four cross-arm ends are intentionally simple solid
    # proxies for hollow pan sockets; the child pan stems are allowed to nest
    # inside them in run_tests().
    for i, (xyz, _yaw) in enumerate(PAN_STATIONS):
        mast.visual(
            Cylinder(radius=0.085, length=0.17),
            origin=Origin(xyz=xyz),
            material=galvanized,
            name=f"socket_{i}",
        )

    for sx in (-0.26, 0.26):
        for sy in (-0.26, 0.26):
            mast.visual(
                Cylinder(radius=0.035, length=0.070),
                origin=Origin(xyz=(sx, sy, 0.430)),
                material=dark_steel,
                name=f"anchor_bolt_{'p' if sx > 0 else 'n'}x_{'p' if sy > 0 else 'n'}y",
            )

    for i, (station_xyz, yaw) in enumerate(PAN_STATIONS):
        knuckle = model.part(f"knuckle_{i}")
        knuckle.visual(
            Cylinder(radius=0.052, length=0.24),
            origin=Origin(xyz=(0.0, 0.0, -0.080)),
            material=galvanized,
            name="stem",
        )
        knuckle.visual(
            Cylinder(radius=0.115, length=0.080),
            origin=Origin(xyz=(0.0, 0.0, -0.240)),
            material=galvanized,
            name="pan_collar",
        )
        knuckle.visual(
            Box((0.18, 0.72, 0.080)),
            origin=Origin(xyz=(0.0, 0.18, -0.210)),
            material=galvanized,
            name="knuckle_neck",
        )
        knuckle.visual(
            Box((0.78, 0.12, 0.070)),
            origin=Origin(xyz=(0.0, 0.36, -0.180)),
            material=galvanized,
            name="yoke_bridge",
        )
        for side, x in enumerate((-0.34, 0.34)):
            knuckle.visual(
                Box((0.060, 0.130, 0.590)),
                origin=Origin(xyz=(x, 0.36, -0.480)),
                material=galvanized,
                name=f"yoke_cheek_{side}",
            )
            knuckle.visual(
                Cylinder(radius=0.072, length=0.075),
                origin=Origin(xyz=(x, 0.36, -0.480), rpy=(0.0, pi / 2.0, 0.0)),
                material=dark_steel,
                name=f"bushing_{side}",
            )

        head = model.part(f"head_{i}")
        head.visual(
            Cylinder(radius=0.050, length=0.74),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_steel,
            name="trunnion",
        )
        head.visual(
            Box((0.56, 0.24, 0.36)),
            origin=Origin(xyz=(0.0, 0.140, 0.0)),
            material=dark_steel,
            name="housing",
        )
        head.visual(
            Box((0.50, 0.024, 0.300)),
            origin=Origin(xyz=(0.0, 0.272, 0.0)),
            material=glass,
            name="lens",
        )
        for col, x in enumerate((-0.18, -0.06, 0.06, 0.18)):
            for row, z in enumerate((-0.085, 0.0, 0.085)):
                head.visual(
                    Box((0.052, 0.010, 0.038)),
                    origin=Origin(xyz=(x, 0.287, z)),
                    material=led,
                    name=f"led_{row}_{col}",
                )
        for fin, x in enumerate((-0.21, -0.14, -0.07, 0.0, 0.07, 0.14, 0.21)):
            head.visual(
                Box((0.020, 0.095, 0.300)),
                origin=Origin(xyz=(x, -0.026, 0.0)),
                material=galvanized,
                name=f"rear_fin_{fin}",
            )
        head.visual(
            Cylinder(radius=0.030, length=0.070),
            origin=Origin(xyz=(0.0, -0.015, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name="cable_gland",
        )

        model.articulation(
            f"pan_{i}",
            ArticulationType.REVOLUTE,
            parent=mast,
            child=knuckle,
            origin=Origin(xyz=station_xyz, rpy=(0.0, 0.0, yaw)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=120.0, velocity=0.7, lower=-1.20, upper=1.20),
        )
        model.articulation(
            f"tilt_{i}",
            ArticulationType.REVOLUTE,
            parent=knuckle,
            child=head,
            origin=Origin(xyz=TILT_ORIGIN),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=80.0, velocity=0.6, lower=-0.85, upper=0.65),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    mast = object_model.get_part("mast")

    ctx.check(
        "four independently panning knuckles",
        all(object_model.get_articulation(f"pan_{i}").articulation_type == ArticulationType.REVOLUTE for i in range(4)),
        details="Expected four vertical revolute pan joints at the cross-arm ends.",
    )
    ctx.check(
        "four independently tilting heads",
        all(object_model.get_articulation(f"tilt_{i}").articulation_type == ArticulationType.REVOLUTE for i in range(4)),
        details="Expected four horizontal revolute tilt joints between each yoke and LED head.",
    )

    for i in range(4):
        knuckle = object_model.get_part(f"knuckle_{i}")
        head = object_model.get_part(f"head_{i}")
        pan = object_model.get_articulation(f"pan_{i}")
        tilt = object_model.get_articulation(f"tilt_{i}")

        ctx.allow_overlap(
            knuckle,
            mast,
            elem_a="stem",
            elem_b=f"socket_{i}",
            reason="The pan stem is intentionally represented seated inside a simplified solid collar socket.",
        )
        ctx.expect_within(
            knuckle,
            mast,
            axes="xy",
            inner_elem="stem",
            outer_elem=f"socket_{i}",
            margin=0.002,
            name=f"pan stem {i} centered in socket",
        )
        ctx.expect_overlap(
            knuckle,
            mast,
            axes="z",
            elem_a="stem",
            elem_b=f"socket_{i}",
            min_overlap=0.060,
            name=f"pan stem {i} retained in socket",
        )

        for side in range(2):
            ctx.allow_overlap(
                head,
                knuckle,
                elem_a="trunnion",
                elem_b=f"bushing_{side}",
                reason="The tilt trunnion is intentionally captured inside the yoke bushing proxy.",
            )
            ctx.allow_overlap(
                head,
                knuckle,
                elem_a="trunnion",
                elem_b=f"yoke_cheek_{side}",
                reason="The yoke cheek is a solid proxy for a bored plate with the trunnion pin passing through it.",
            )
            ctx.expect_overlap(
                head,
                knuckle,
                axes="xyz",
                elem_a="trunnion",
                elem_b=f"bushing_{side}",
                min_overlap=0.040,
                name=f"trunnion {i}-{side} passes through bushing",
            )
            ctx.expect_overlap(
                head,
                knuckle,
                axes="xyz",
                elem_a="trunnion",
                elem_b=f"yoke_cheek_{side}",
                min_overlap=0.040,
                name=f"trunnion {i}-{side} passes through yoke cheek",
            )

        ctx.check(
            f"pan {i} vertical axis",
            tuple(round(v, 6) for v in pan.axis) == (0.0, 0.0, 1.0),
            details=f"axis={pan.axis}",
        )
        ctx.check(
            f"tilt {i} horizontal axis",
            tuple(round(v, 6) for v in tilt.axis) == (1.0, 0.0, 0.0),
            details=f"axis={tilt.axis}",
        )

        rest_lens = ctx.part_element_world_aabb(head, elem="lens")
        with ctx.pose({tilt: -0.55}):
            tilted_lens = ctx.part_element_world_aabb(head, elem="lens")
        if rest_lens is not None and tilted_lens is not None:
            rest_z = (rest_lens[0][2] + rest_lens[1][2]) * 0.5
            tilted_z = (tilted_lens[0][2] + tilted_lens[1][2]) * 0.5
            ctx.check(
                f"head {i} tilts downward",
                tilted_z < rest_z - 0.050,
                details=f"rest_lens_z={rest_z}, tilted_lens_z={tilted_z}",
            )
        else:
            ctx.fail(f"head {i} tilt measurement", "Could not measure lens AABB.")

    head_0 = object_model.get_part("head_0")
    pan_0 = object_model.get_articulation("pan_0")
    rest_pos = ctx.part_world_position(head_0)
    with ctx.pose({pan_0: 0.60}):
        panned_pos = ctx.part_world_position(head_0)
    ctx.check(
        "pan joint swings head around vertical mast axis",
        rest_pos is not None
        and panned_pos is not None
        and abs(panned_pos[2] - rest_pos[2]) < 0.010
        and ((panned_pos[0] - rest_pos[0]) ** 2 + (panned_pos[1] - rest_pos[1]) ** 2) ** 0.5 > 0.15,
        details=f"rest={rest_pos}, panned={panned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
