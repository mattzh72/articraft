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


PALM_BODY_SIZE = (0.094, 0.180, 0.028)
PALM_TOP_CAP_SIZE = (0.070, 0.128, 0.008)
PALM_FRONT_RAIL_SIZE = (0.094, 0.024, 0.010)
PALM_FRONT_FACE_X = PALM_BODY_SIZE[0] * 0.5
PALM_TOP_Z = PALM_BODY_SIZE[2] + PALM_TOP_CAP_SIZE[2]
FINGER_BARREL_RADIUS = 0.0055
FINGER_BARREL_LENGTH = 0.022


def _build_finger_chain(
    part,
    *,
    finger_material,
    pad_material,
    width: float,
    proximal_len: float,
    middle_len: float,
    distal_len: float,
) -> None:
    prox_height = 0.012
    mid_height = 0.010
    dist_height = 0.008
    tip_radius = 0.0038

    prox_start = FINGER_BARREL_RADIUS - 0.0015
    mid_start = prox_start + proximal_len - 0.0012
    dist_start = mid_start + middle_len - 0.0010

    part.visual(
        Cylinder(radius=FINGER_BARREL_RADIUS, length=FINGER_BARREL_LENGTH),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=finger_material,
        name="hinge_barrel",
    )
    part.visual(
        Box((proximal_len, width * 0.88, prox_height)),
        origin=Origin(xyz=(prox_start + proximal_len * 0.5, 0.0, 0.0)),
        material=finger_material,
        name="proximal_link",
    )
    part.visual(
        Box((middle_len, width * 0.78, mid_height)),
        origin=Origin(xyz=(mid_start + middle_len * 0.5, 0.0, 0.0)),
        material=finger_material,
        name="middle_link",
    )
    part.visual(
        Box((distal_len, width * 0.70, dist_height)),
        origin=Origin(xyz=(dist_start + distal_len * 0.5, 0.0, 0.0)),
        material=finger_material,
        name="distal_link",
    )
    part.visual(
        Box((distal_len * 0.78, width * 0.58, 0.0055)),
        origin=Origin(
            xyz=(dist_start + distal_len * 0.54, 0.0, -0.0010),
        ),
        material=pad_material,
        name="distal_pad",
    )
    part.visual(
        Cylinder(radius=tip_radius, length=width * 0.58),
        origin=Origin(
            xyz=(dist_start + distal_len - 0.0010 + tip_radius, 0.0, -0.0005),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=pad_material,
        name="finger_tip",
    )


def _build_sensor_cover(part, *, cover_material, hinge_material, window_material) -> None:
    part.visual(
        Cylinder(radius=0.0018, length=0.022),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=hinge_material,
        name="cover_hinge",
    )
    part.visual(
        Box((0.032, 0.028, 0.003)),
        origin=Origin(xyz=(0.016, 0.0, -0.0003)),
        material=cover_material,
        name="cover_panel",
    )
    part.visual(
        Box((0.018, 0.012, 0.0012)),
        origin=Origin(xyz=(0.018, 0.0, 0.0017)),
        material=window_material,
        name="cover_window",
    )
    part.visual(
        Box((0.006, 0.010, 0.0018)),
        origin=Origin(xyz=(0.0305, 0.0, 0.0021)),
        material=hinge_material,
        name="cover_lip",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="palm_module")

    housing = model.material("housing", rgba=(0.18, 0.20, 0.23, 1.0))
    housing_cap = model.material("housing_cap", rgba=(0.24, 0.26, 0.29, 1.0))
    finger_shell = model.material("finger_shell", rgba=(0.27, 0.29, 0.31, 1.0))
    finger_pad = model.material("finger_pad", rgba=(0.12, 0.13, 0.14, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.56, 0.58, 0.60, 1.0))
    cover_finish = model.material("cover_finish", rgba=(0.35, 0.37, 0.40, 1.0))
    sensor_glass = model.material("sensor_glass", rgba=(0.18, 0.38, 0.48, 0.72))

    palm = model.part("palm")
    palm.visual(
        Box(PALM_BODY_SIZE),
        origin=Origin(xyz=(0.0, 0.0, PALM_BODY_SIZE[2] * 0.5)),
        material=housing,
        name="body_block",
    )
    palm.visual(
        Box(PALM_TOP_CAP_SIZE),
        origin=Origin(xyz=(-0.004, 0.0, PALM_BODY_SIZE[2] + PALM_TOP_CAP_SIZE[2] * 0.5)),
        material=housing_cap,
        name="top_cap",
    )
    palm.visual(
        Box(PALM_FRONT_RAIL_SIZE),
        origin=Origin(
            xyz=(
                PALM_FRONT_FACE_X - PALM_FRONT_RAIL_SIZE[0] * 0.5,
                0.0,
                PALM_BODY_SIZE[2] + PALM_FRONT_RAIL_SIZE[2] * 0.5 - 0.002,
            )
        ),
        material=housing_cap,
        name="finger_mount_rail",
    )
    palm.inertial = Inertial.from_geometry(
        Box((0.094, 0.180, 0.036)),
        mass=0.95,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
    )

    finger_specs = [
        {"name": "finger_0", "y": -0.057, "width": 0.020, "prox": 0.027, "mid": 0.020, "dist": 0.016},
        {"name": "finger_1", "y": -0.019, "width": 0.022, "prox": 0.030, "mid": 0.022, "dist": 0.018},
        {"name": "finger_2", "y": 0.019, "width": 0.022, "prox": 0.030, "mid": 0.022, "dist": 0.018},
        {"name": "finger_3", "y": 0.057, "width": 0.020, "prox": 0.027, "mid": 0.020, "dist": 0.016},
    ]
    for index, spec in enumerate(finger_specs):
        finger = model.part(spec["name"])
        _build_finger_chain(
            finger,
            finger_material=finger_shell,
            pad_material=finger_pad,
            width=spec["width"],
            proximal_len=spec["prox"],
            middle_len=spec["mid"],
            distal_len=spec["dist"],
        )
        finger.inertial = Inertial.from_geometry(
            Box((spec["prox"] + spec["mid"] + spec["dist"] + 0.012, spec["width"], 0.016)),
            mass=0.085 if index in (1, 2) else 0.075,
            origin=Origin(
                xyz=(
                    (spec["prox"] + spec["mid"] + spec["dist"]) * 0.5 + 0.004,
                    0.0,
                    0.0,
                )
            ),
        )
        model.articulation(
            f"palm_to_{spec['name']}",
            ArticulationType.REVOLUTE,
            parent=palm,
            child=finger,
            origin=Origin(
                xyz=(
                    PALM_FRONT_FACE_X + FINGER_BARREL_RADIUS,
                    spec["y"],
                    0.018,
                )
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.5,
                velocity=2.0,
                lower=0.0,
                upper=1.10,
            ),
        )

    cover = model.part("sensor_cover")
    _build_sensor_cover(
        cover,
        cover_material=cover_finish,
        hinge_material=hinge_steel,
        window_material=sensor_glass,
    )
    cover.inertial = Inertial.from_geometry(
        Box((0.032, 0.028, 0.004)),
        mass=0.028,
        origin=Origin(xyz=(0.016, 0.0, 0.0004)),
    )
    model.articulation(
        "palm_to_sensor_cover",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=cover,
        origin=Origin(xyz=(-0.016, 0.0, PALM_TOP_Z + 0.0018)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=2.5,
            lower=0.0,
            upper=1.20,
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
    top_cap = palm.get_visual("top_cap")
    sensor_cover = object_model.get_part("sensor_cover")
    cover_panel = sensor_cover.get_visual("cover_panel")
    cover_joint = object_model.get_articulation("palm_to_sensor_cover")

    finger_parts = [object_model.get_part(f"finger_{index}") for index in range(4)]
    finger_joints = [object_model.get_articulation(f"palm_to_finger_{index}") for index in range(4)]

    for index, joint in enumerate(finger_joints):
        limits = joint.motion_limits
        ctx.check(
            f"finger_{index} base articulation is configured",
            joint.articulation_type == ArticulationType.REVOLUTE
            and joint.axis == (0.0, -1.0, 0.0)
            and limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and limits.upper >= 1.0,
            details=f"type={joint.articulation_type}, axis={joint.axis}, limits={limits}",
        )
        ctx.expect_gap(
            finger_parts[index],
            palm,
            axis="x",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"finger_{index} root seats on the palm front face",
        )
        ctx.expect_overlap(
            finger_parts[index],
            palm,
            axes="yz",
            min_overlap=0.008,
            name=f"finger_{index} root overlaps the palm mount zone",
        )

    ctx.expect_gap(
        sensor_cover,
        palm,
        axis="z",
        positive_elem=cover_panel,
        negative_elem=top_cap,
        max_gap=0.001,
        max_penetration=0.0,
        name="sensor cover sits flush on the palm top face",
    )
    ctx.expect_overlap(
        sensor_cover,
        palm,
        axes="xy",
        elem_a=cover_panel,
        elem_b=top_cap,
        min_overlap=0.020,
        name="sensor cover stays centered on the palm top face",
    )

    for index, joint in enumerate(finger_joints):
        rest_tip = ctx.part_element_world_aabb(finger_parts[index], elem="distal_pad")
        with ctx.pose({joint: 0.72}):
            raised_tip = ctx.part_element_world_aabb(finger_parts[index], elem="distal_pad")
        ctx.check(
            f"finger_{index} lifts when the base rotates",
            rest_tip is not None
            and raised_tip is not None
            and raised_tip[1][2] > rest_tip[1][2] + 0.015,
            details=f"rest_tip={rest_tip}, raised_tip={raised_tip}",
        )

    rest_cover = ctx.part_element_world_aabb(sensor_cover, elem="cover_panel")
    with ctx.pose({cover_joint: 0.95}):
        open_cover = ctx.part_element_world_aabb(sensor_cover, elem="cover_panel")
    ctx.check(
        "sensor cover folds upward from the palm face",
        rest_cover is not None
        and open_cover is not None
        and open_cover[1][2] > rest_cover[1][2] + 0.012,
        details=f"rest_cover={rest_cover}, open_cover={open_cover}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
