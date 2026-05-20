from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


PALM_WIDTH = 0.110
PALM_DEPTH = 0.070
PALM_THICKNESS = 0.028
FINGER_BOSS_DEPTH = 0.018
FINGER_BOSS_HEIGHT = 0.012
FINGER_BOSS_Y = 0.028
FINGER_BOSS_TOP_Z = PALM_THICKNESS + FINGER_BOSS_HEIGHT


def _add_regular_segment(
    part,
    *,
    body_name: str,
    barrel_name: str,
    length: float,
    width: float,
    thickness: float,
    barrel_radius: float,
    material,
    mass: float,
    tip_cap: bool = False,
) -> None:
    part.visual(
        Cylinder(radius=barrel_radius, length=width),
        origin=Origin(
            xyz=(0.0, barrel_radius, barrel_radius),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=material,
        name=barrel_name,
    )
    part.visual(
        Box((width, length, thickness)),
        origin=Origin(xyz=(0.0, barrel_radius + length / 2.0, thickness / 2.0)),
        material=material,
        name=body_name,
    )
    if tip_cap:
        tip_radius = min(thickness * 0.42, width * 0.42)
        part.visual(
            Cylinder(radius=tip_radius, length=width * 0.92),
            origin=Origin(
                xyz=(0.0, barrel_radius + length, thickness / 2.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=material,
            name=f"{body_name}_tip",
        )
    part.inertial = Inertial.from_geometry(
        Box((width, length + barrel_radius, thickness)),
        mass=mass,
        origin=Origin(
            xyz=(0.0, (length + barrel_radius) / 2.0, thickness / 2.0),
        ),
    )


def _add_thumb_segment(
    part,
    *,
    body_name: str,
    barrel_name: str,
    length: float,
    width: float,
    thickness: float,
    barrel_radius: float,
    material,
    mass: float,
    tip_cap: bool = False,
) -> None:
    part.visual(
        Cylinder(radius=barrel_radius, length=width),
        origin=Origin(
            xyz=(barrel_radius, 0.0, barrel_radius),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=material,
        name=barrel_name,
    )
    part.visual(
        Box((length, width, thickness)),
        origin=Origin(xyz=(barrel_radius + length / 2.0, 0.0, thickness / 2.0)),
        material=material,
        name=body_name,
    )
    if tip_cap:
        tip_radius = min(thickness * 0.42, width * 0.42)
        part.visual(
            Cylinder(radius=tip_radius, length=width * 0.92),
            origin=Origin(
                xyz=(barrel_radius + length, 0.0, thickness / 2.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=material,
            name=f"{body_name}_tip",
        )
    part.inertial = Inertial.from_geometry(
        Box((length + barrel_radius, width, thickness)),
        mass=mass,
        origin=Origin(
            xyz=((length + barrel_radius) / 2.0, 0.0, thickness / 2.0),
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dexterous_palm")

    palm_shell = model.material("palm_shell", rgba=(0.26, 0.28, 0.31, 1.0))
    knuckle_shell = model.material("knuckle_shell", rgba=(0.56, 0.59, 0.63, 1.0))
    fingertip_shell = model.material("fingertip_shell", rgba=(0.68, 0.71, 0.74, 1.0))
    thumb_shell = model.material("thumb_shell", rgba=(0.60, 0.63, 0.66, 1.0))

    palm = model.part("palm")
    palm.visual(
        Box((PALM_WIDTH, PALM_DEPTH, PALM_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, PALM_THICKNESS / 2.0)),
        material=palm_shell,
        name="palm_block",
    )
    palm.visual(
        Box((0.064, 0.026, 0.016)),
        origin=Origin(xyz=(0.0, -0.023, PALM_THICKNESS + 0.008)),
        material=palm_shell,
        name="wrist_riser",
    )
    palm.visual(
        Box((0.022, 0.020, 0.012)),
        origin=Origin(xyz=(0.0, -0.012, PALM_THICKNESS + 0.006)),
        material=knuckle_shell,
        name="central_bridge",
    )

    finger_boss_specs = {
        "little_boss": (-0.036, 0.012),
        "ring_boss": (-0.014, 0.014),
        "index_boss": (0.012, 0.015),
        "middle_boss": (0.036, 0.015),
    }
    for boss_name, (x_pos, boss_width) in finger_boss_specs.items():
        palm.visual(
            Box((boss_width, FINGER_BOSS_DEPTH, FINGER_BOSS_HEIGHT)),
            origin=Origin(xyz=(x_pos, FINGER_BOSS_Y, PALM_THICKNESS + FINGER_BOSS_HEIGHT / 2.0)),
            material=knuckle_shell,
            name=boss_name,
        )

    palm.visual(
        Box((0.018, 0.024, 0.016)),
        origin=Origin(
            xyz=(PALM_WIDTH / 2.0 + 0.005, 0.002, PALM_THICKNESS + 0.008),
        ),
        material=knuckle_shell,
        name="thumb_boss",
    )
    palm.inertial = Inertial.from_geometry(
        Box((0.122, 0.080, 0.044)),
        mass=0.65,
        origin=Origin(xyz=(0.005, 0.0, 0.022)),
    )

    finger_specs = {
        "little": {
            "x": -0.036,
            "yaw": 0.30,
            "widths": (0.012, 0.011, 0.010),
            "thicknesses": (0.014, 0.013, 0.012),
            "lengths": (0.030, 0.022, 0.017),
            "radii": (0.0048, 0.0043, 0.0040),
            "masses": (0.050, 0.038, 0.028),
        },
        "ring": {
            "x": -0.014,
            "yaw": 0.12,
            "widths": (0.014, 0.013, 0.012),
            "thicknesses": (0.015, 0.014, 0.013),
            "lengths": (0.036, 0.027, 0.020),
            "radii": (0.0052, 0.0046, 0.0042),
            "masses": (0.060, 0.046, 0.034),
        },
        "index": {
            "x": 0.012,
            "yaw": -0.05,
            "widths": (0.015, 0.014, 0.013),
            "thicknesses": (0.016, 0.015, 0.013),
            "lengths": (0.038, 0.028, 0.021),
            "radii": (0.0054, 0.0048, 0.0043),
            "masses": (0.064, 0.050, 0.036),
        },
        "middle": {
            "x": 0.036,
            "yaw": -0.18,
            "widths": (0.016, 0.015, 0.014),
            "thicknesses": (0.017, 0.015, 0.014),
            "lengths": (0.042, 0.030, 0.023),
            "radii": (0.0057, 0.0050, 0.0045),
            "masses": (0.070, 0.054, 0.040),
        },
    }

    for finger_name, spec in finger_specs.items():
        proximal = model.part(f"{finger_name}_proximal")
        middle = model.part(f"{finger_name}_middle")
        distal = model.part(f"{finger_name}_distal")

        _add_regular_segment(
            proximal,
            body_name=f"{finger_name}_proximal_body",
            barrel_name=f"{finger_name}_proximal_barrel",
            length=spec["lengths"][0],
            width=spec["widths"][0],
            thickness=spec["thicknesses"][0],
            barrel_radius=spec["radii"][0],
            material=knuckle_shell,
            mass=spec["masses"][0],
        )
        _add_regular_segment(
            middle,
            body_name=f"{finger_name}_middle_body",
            barrel_name=f"{finger_name}_middle_barrel",
            length=spec["lengths"][1],
            width=spec["widths"][1],
            thickness=spec["thicknesses"][1],
            barrel_radius=spec["radii"][1],
            material=knuckle_shell,
            mass=spec["masses"][1],
        )
        _add_regular_segment(
            distal,
            body_name=f"{finger_name}_distal_body",
            barrel_name=f"{finger_name}_distal_barrel",
            length=spec["lengths"][2],
            width=spec["widths"][2],
            thickness=spec["thicknesses"][2],
            barrel_radius=spec["radii"][2],
            material=fingertip_shell,
            mass=spec["masses"][2],
            tip_cap=True,
        )

        model.articulation(
            f"palm_to_{finger_name}_base",
            ArticulationType.REVOLUTE,
            parent=palm,
            child=proximal,
            origin=Origin(
                xyz=(spec["x"], FINGER_BOSS_Y, FINGER_BOSS_TOP_Z),
                rpy=(0.0, 0.0, spec["yaw"]),
            ),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=3.0,
                lower=0.0,
                upper=1.35,
            ),
        )
        model.articulation(
            f"{finger_name}_proximal_to_{finger_name}_middle",
            ArticulationType.REVOLUTE,
            parent=proximal,
            child=middle,
            origin=Origin(xyz=(0.0, spec["lengths"][0] + spec["radii"][0], 0.0)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=3.0,
                lower=0.0,
                upper=1.45,
            ),
        )
        model.articulation(
            f"{finger_name}_middle_to_{finger_name}_distal",
            ArticulationType.REVOLUTE,
            parent=middle,
            child=distal,
            origin=Origin(xyz=(0.0, spec["lengths"][1] + spec["radii"][1], 0.0)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=3.2,
                lower=0.0,
                upper=1.20,
            ),
        )

    thumb_proximal = model.part("thumb_proximal")
    thumb_distal = model.part("thumb_distal")
    _add_thumb_segment(
        thumb_proximal,
        body_name="thumb_proximal_body",
        barrel_name="thumb_proximal_barrel",
        length=0.031,
        width=0.018,
        thickness=0.016,
        barrel_radius=0.0056,
        material=thumb_shell,
        mass=0.062,
    )
    _add_thumb_segment(
        thumb_distal,
        body_name="thumb_distal_body",
        barrel_name="thumb_distal_barrel",
        length=0.024,
        width=0.016,
        thickness=0.014,
        barrel_radius=0.0048,
        material=fingertip_shell,
        mass=0.040,
        tip_cap=True,
    )

    model.articulation(
        "palm_to_thumb_base",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=thumb_proximal,
        origin=Origin(
            xyz=(PALM_WIDTH / 2.0 + 0.005, 0.002, PALM_THICKNESS + 0.016),
            rpy=(0.0, 0.0, 0.72),
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.5,
            velocity=2.5,
            lower=-0.35,
            upper=1.00,
        ),
    )
    model.articulation(
        "thumb_proximal_to_thumb_distal",
        ArticulationType.REVOLUTE,
        parent=thumb_proximal,
        child=thumb_distal,
        origin=Origin(xyz=(0.031 + 0.0056, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.2,
            velocity=2.8,
            lower=0.0,
            upper=1.25,
        ),
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

    palm = object_model.get_part("palm")
    little_proximal = object_model.get_part("little_proximal")
    ring_proximal = object_model.get_part("ring_proximal")
    index_proximal = object_model.get_part("index_proximal")
    middle_proximal = object_model.get_part("middle_proximal")
    index_distal = object_model.get_part("index_distal")
    thumb_proximal = object_model.get_part("thumb_proximal")
    thumb_distal = object_model.get_part("thumb_distal")

    base_joint_names = (
        "palm_to_little_base",
        "palm_to_ring_base",
        "palm_to_index_base",
        "palm_to_middle_base",
    )
    base_joints = [object_model.get_articulation(name) for name in base_joint_names]
    thumb_base = object_model.get_articulation("palm_to_thumb_base")
    index_base = object_model.get_articulation("palm_to_index_base")
    index_mid = object_model.get_articulation("index_proximal_to_index_middle")
    index_tip = object_model.get_articulation("index_middle_to_index_distal")
    thumb_tip = object_model.get_articulation("thumb_proximal_to_thumb_distal")

    for finger_part, boss_name in (
        (little_proximal, "little_boss"),
        (ring_proximal, "ring_boss"),
        (index_proximal, "index_boss"),
        (middle_proximal, "middle_boss"),
    ):
        ctx.expect_gap(
            finger_part,
            palm,
            axis="z",
            max_gap=0.0005,
            max_penetration=1e-6,
            negative_elem=boss_name,
            name=f"{finger_part.name} sits on its boss",
        )
        ctx.expect_overlap(
            finger_part,
            palm,
            axes="xy",
            min_overlap=0.006,
            elem_b=boss_name,
            name=f"{finger_part.name} overlaps its mounting boss footprint",
        )

    ctx.expect_gap(
        thumb_proximal,
        palm,
        axis="z",
        max_gap=0.0005,
        max_penetration=1e-6,
        negative_elem="thumb_boss",
        name="thumb sits on raised side boss",
    )
    ctx.expect_overlap(
        thumb_proximal,
        palm,
        axes="xy",
        min_overlap=0.006,
        elem_b="thumb_boss",
        name="thumb overlaps side boss footprint",
    )

    base_origins = [joint.origin.xyz for joint in base_joints]
    distinct_origins = {
        tuple(round(value, 4) for value in origin)
        for origin in base_origins
    }
    ctx.check(
        "central and side fingers have independent base joints",
        len(distinct_origins) == 4 and len({round(origin[0], 4) for origin in base_origins}) == 4,
        details=f"base_origins={base_origins}",
    )
    ctx.check(
        "thumb base is mounted on the palm side",
        thumb_base.origin.xyz[0] > PALM_WIDTH * 0.45 and abs(thumb_base.origin.xyz[1]) < 0.015,
        details=f"thumb_origin={thumb_base.origin.xyz}",
    )

    rest_index_pos = ctx.part_world_position(index_distal)
    with ctx.pose(
        {
            index_base: 0.75,
            index_mid: 1.00,
            index_tip: 0.85,
        }
    ):
        curled_index_pos = ctx.part_world_position(index_distal)
    ctx.check(
        "index finger curls downward under positive flexion",
        rest_index_pos is not None
        and curled_index_pos is not None
        and curled_index_pos[2] < rest_index_pos[2] - 0.010,
        details=f"rest={rest_index_pos}, curled={curled_index_pos}",
    )

    rest_thumb_pos = ctx.part_world_position(thumb_distal)
    with ctx.pose(
        {
            thumb_base: 0.70,
            thumb_tip: 0.85,
        }
    ):
        curled_thumb_pos = ctx.part_world_position(thumb_distal)
    ctx.check(
        "thumb chain curls down from the side boss",
        rest_thumb_pos is not None
        and curled_thumb_pos is not None
        and curled_thumb_pos[2] < rest_thumb_pos[2] - 0.008,
        details=f"rest={rest_thumb_pos}, curled={curled_thumb_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
