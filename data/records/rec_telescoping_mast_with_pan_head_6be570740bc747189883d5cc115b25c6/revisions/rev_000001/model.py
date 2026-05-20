from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


BASE_RADIUS = 0.17
BASE_THICKNESS = 0.025
PEDESTAL_RADIUS = 0.032
PEDESTAL_HEIGHT = 0.055

BASE_SLEEVE_OUTER = 0.090
BASE_SLEEVE_INNER = 0.078
BASE_SLEEVE_HEIGHT = 0.720
BASE_GUIDE_OUTER = 0.102
BASE_GUIDE_INNER = 0.074
BASE_GUIDE_HEIGHT = 0.030

STAGE1_OUTER = 0.072
STAGE1_INNER = 0.063
STAGE1_LENGTH = 0.620
STAGE1_INSERT = 0.400
STAGE1_COLLAR_OUTER = 0.094
STAGE1_COLLAR_HEIGHT = 0.024
STAGE1_GUIDE_OUTER = 0.082
STAGE1_GUIDE_INNER = 0.060
STAGE1_GUIDE_HEIGHT = 0.028
STAGE1_TRAVEL = 0.200

STAGE2_OUTER = 0.058
STAGE2_INNER = 0.050
STAGE2_LENGTH = 0.520
STAGE2_INSERT = 0.340
STAGE2_COLLAR_OUTER = 0.078
STAGE2_COLLAR_HEIGHT = 0.022
STAGE2_GUIDE_OUTER = 0.068
STAGE2_GUIDE_INNER = 0.046
STAGE2_GUIDE_HEIGHT = 0.026
STAGE2_TRAVEL = 0.170

STAGE3_OUTER = 0.044
STAGE3_INNER = 0.036
STAGE3_LENGTH = 0.420
STAGE3_INSERT = 0.280
STAGE3_COLLAR_OUTER = 0.064
STAGE3_COLLAR_HEIGHT = 0.020
STAGE3_TOP_PLATE_DIAMETER = 0.056
STAGE3_TOP_PLATE_HEIGHT = 0.018
STAGE3_TRAVEL = 0.140
def add_square_ring_visuals(
    part,
    *,
    outer: float,
    inner: float,
    z0: float,
    height: float,
    material,
    prefix: str,
) -> None:
    wall = 0.5 * (outer - inner)
    center_offset = 0.25 * (outer + inner)
    center_z = z0 + 0.5 * height

    part.visual(
        Box((outer, wall, height)),
        origin=Origin(xyz=(0.0, center_offset, center_z)),
        material=material,
        name=f"{prefix}_north",
    )
    part.visual(
        Box((outer, wall, height)),
        origin=Origin(xyz=(0.0, -center_offset, center_z)),
        material=material,
        name=f"{prefix}_south",
    )
    part.visual(
        Box((wall, outer, height)),
        origin=Origin(xyz=(center_offset, 0.0, center_z)),
        material=material,
        name=f"{prefix}_east",
    )
    part.visual(
        Box((wall, outer, height)),
        origin=Origin(xyz=(-center_offset, 0.0, center_z)),
        material=material,
        name=f"{prefix}_west",
    )


def add_stage_geometry(
    part,
    *,
    outer: float,
    inner: float,
    tube_length: float,
    insert_depth: float,
    collar_outer: float,
    collar_height: float,
    guide_outer: float | None,
    guide_inner: float | None,
    guide_height: float | None,
    material,
    prefix: str,
    top_plate_diameter: float | None = None,
    top_plate_height: float | None = None,
) -> float:
    exposed_height = tube_length - insert_depth
    add_square_ring_visuals(
        part,
        outer=outer,
        inner=inner,
        z0=-insert_depth,
        height=tube_length,
        material=material,
        prefix=f"{prefix}_tube",
    )
    add_square_ring_visuals(
        part,
        outer=collar_outer,
        inner=outer,
        z0=0.0,
        height=collar_height,
        material=material,
        prefix=f"{prefix}_collar",
    )
    top_z = exposed_height
    if guide_outer is not None and guide_inner is not None and guide_height is not None:
        add_square_ring_visuals(
            part,
            outer=guide_outer,
            inner=guide_inner,
            z0=exposed_height,
            height=guide_height,
            material=material,
            prefix=f"{prefix}_guide",
        )
        top_z = exposed_height + guide_height
    if top_plate_diameter is not None and top_plate_height is not None:
        part.visual(
            Cylinder(radius=0.5 * top_plate_diameter, length=top_plate_height),
            origin=Origin(xyz=(0.0, 0.0, exposed_height + 0.5 * top_plate_height)),
            material=material,
            name=f"{prefix}_top_plate",
        )
        top_z = exposed_height + top_plate_height
    return top_z


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_mast_with_pan_head")

    base_black = model.material("base_black", rgba=(0.12, 0.12, 0.14, 1.0))
    mast_aluminum = model.material("mast_aluminum", rgba=(0.74, 0.77, 0.80, 1.0))
    head_black = model.material("head_black", rgba=(0.18, 0.18, 0.20, 1.0))
    plate_gray = model.material("plate_gray", rgba=(0.56, 0.58, 0.61, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * BASE_THICKNESS)),
        material=base_black,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=PEDESTAL_RADIUS, length=PEDESTAL_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + 0.5 * PEDESTAL_HEIGHT)),
        material=base_black,
        name="pedestal",
    )
    rib_z0 = BASE_THICKNESS + 0.020
    rib_height = PEDESTAL_HEIGHT - 0.020
    rib_center_z = rib_z0 + 0.5 * rib_height
    radial_bridge = 0.5 * BASE_SLEEVE_INNER - PEDESTAL_RADIUS + 0.012
    rib_center = 0.5 * (PEDESTAL_RADIUS + 0.5 * BASE_SLEEVE_INNER)
    base.visual(
        Box((0.050, radial_bridge, rib_height)),
        origin=Origin(xyz=(0.0, rib_center, rib_center_z)),
        material=base_black,
        name="front_rib",
    )
    base.visual(
        Box((0.050, radial_bridge, rib_height)),
        origin=Origin(xyz=(0.0, -rib_center, rib_center_z)),
        material=base_black,
        name="rear_rib",
    )
    base.visual(
        Box((radial_bridge, 0.050, rib_height)),
        origin=Origin(xyz=(rib_center, 0.0, rib_center_z)),
        material=base_black,
        name="right_rib",
    )
    base.visual(
        Box((radial_bridge, 0.050, rib_height)),
        origin=Origin(xyz=(-rib_center, 0.0, rib_center_z)),
        material=base_black,
        name="left_rib",
    )
    add_square_ring_visuals(
        base,
        outer=BASE_SLEEVE_OUTER,
        inner=BASE_SLEEVE_INNER,
        z0=BASE_THICKNESS + 0.020,
        height=BASE_SLEEVE_HEIGHT,
        material=base_black,
        prefix="base_sleeve",
    )
    add_square_ring_visuals(
        base,
        outer=BASE_GUIDE_OUTER,
        inner=BASE_GUIDE_INNER,
        z0=BASE_THICKNESS + 0.020 + BASE_SLEEVE_HEIGHT,
        height=BASE_GUIDE_HEIGHT,
        material=base_black,
        prefix="base_guide",
    )

    mast_stage_1 = model.part("mast_stage_1")
    stage1_top_z = add_stage_geometry(
        mast_stage_1,
        outer=STAGE1_OUTER,
        inner=STAGE1_INNER,
        tube_length=STAGE1_LENGTH,
        insert_depth=STAGE1_INSERT,
        collar_outer=STAGE1_COLLAR_OUTER,
        collar_height=STAGE1_COLLAR_HEIGHT,
        guide_outer=STAGE1_GUIDE_OUTER,
        guide_inner=STAGE1_GUIDE_INNER,
        guide_height=STAGE1_GUIDE_HEIGHT,
        material=mast_aluminum,
        prefix="stage_1",
    )

    mast_stage_2 = model.part("mast_stage_2")
    stage2_top_z = add_stage_geometry(
        mast_stage_2,
        outer=STAGE2_OUTER,
        inner=STAGE2_INNER,
        tube_length=STAGE2_LENGTH,
        insert_depth=STAGE2_INSERT,
        collar_outer=STAGE2_COLLAR_OUTER,
        collar_height=STAGE2_COLLAR_HEIGHT,
        guide_outer=STAGE2_GUIDE_OUTER,
        guide_inner=STAGE2_GUIDE_INNER,
        guide_height=STAGE2_GUIDE_HEIGHT,
        material=mast_aluminum,
        prefix="stage_2",
    )

    mast_stage_3 = model.part("mast_stage_3")
    stage3_top_z = add_stage_geometry(
        mast_stage_3,
        outer=STAGE3_OUTER,
        inner=STAGE3_INNER,
        tube_length=STAGE3_LENGTH,
        insert_depth=STAGE3_INSERT,
        collar_outer=STAGE3_COLLAR_OUTER,
        collar_height=STAGE3_COLLAR_HEIGHT,
        guide_outer=None,
        guide_inner=None,
        guide_height=None,
        material=mast_aluminum,
        prefix="stage_3",
        top_plate_diameter=STAGE3_TOP_PLATE_DIAMETER,
        top_plate_height=STAGE3_TOP_PLATE_HEIGHT,
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.050, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=head_black,
        name="turntable",
    )
    pan_head.visual(
        Box((0.060, 0.036, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=head_black,
        name="body_block",
    )
    pan_head.visual(
        Box((0.068, 0.026, 0.028)),
        origin=Origin(xyz=(0.044, 0.0, 0.054)),
        material=head_black,
        name="arm",
    )
    pan_head.visual(
        Box((0.018, 0.090, 0.060)),
        origin=Origin(xyz=(0.079, 0.0, 0.070)),
        material=head_black,
        name="pan_body",
    )
    pan_head.visual(
        Box((0.010, 0.100, 0.070)),
        origin=Origin(xyz=(0.093, 0.0, 0.070)),
        material=plate_gray,
        name="faceplate",
    )

    base_to_stage_1_origin_z = BASE_THICKNESS + 0.020 + BASE_SLEEVE_HEIGHT + BASE_GUIDE_HEIGHT
    model.articulation(
        "base_to_stage_1",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast_stage_1,
        origin=Origin(xyz=(0.0, 0.0, base_to_stage_1_origin_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.30,
            lower=0.0,
            upper=STAGE1_TRAVEL,
        ),
    )
    model.articulation(
        "stage_1_to_stage_2",
        ArticulationType.PRISMATIC,
        parent=mast_stage_1,
        child=mast_stage_2,
        origin=Origin(xyz=(0.0, 0.0, stage1_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.28,
            lower=0.0,
            upper=STAGE2_TRAVEL,
        ),
    )
    model.articulation(
        "stage_2_to_stage_3",
        ArticulationType.PRISMATIC,
        parent=mast_stage_2,
        child=mast_stage_3,
        origin=Origin(xyz=(0.0, 0.0, stage2_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=70.0,
            velocity=0.25,
            lower=0.0,
            upper=STAGE3_TRAVEL,
        ),
    )
    model.articulation(
        "stage_3_to_pan_head",
        ArticulationType.REVOLUTE,
        parent=mast_stage_3,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, stage3_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=-pi,
            upper=pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    mast_stage_1 = object_model.get_part("mast_stage_1")
    mast_stage_2 = object_model.get_part("mast_stage_2")
    mast_stage_3 = object_model.get_part("mast_stage_3")
    pan_head = object_model.get_part("pan_head")

    base_to_stage_1 = object_model.get_articulation("base_to_stage_1")
    stage_1_to_stage_2 = object_model.get_articulation("stage_1_to_stage_2")
    stage_2_to_stage_3 = object_model.get_articulation("stage_2_to_stage_3")
    stage_3_to_pan_head = object_model.get_articulation("stage_3_to_pan_head")

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

    ctx.expect_contact(mast_stage_1, base, name="lower mast seats on grounded base")
    ctx.expect_contact(mast_stage_2, mast_stage_1, name="middle mast seats on lower mast")
    ctx.expect_contact(mast_stage_3, mast_stage_2, name="upper mast seats on middle mast")
    ctx.expect_contact(pan_head, mast_stage_3, name="pan head mounts to top mast plate")

    prismatic_joints = (base_to_stage_1, stage_1_to_stage_2, stage_2_to_stage_3)
    ctx.check(
        "mast lift joints are vertical prismatic stages",
        all(
            joint.articulation_type == ArticulationType.PRISMATIC
            and tuple(round(value, 6) for value in joint.axis) == (0.0, 0.0, 1.0)
            and joint.motion_limits is not None
            and joint.motion_limits.lower == 0.0
            and joint.motion_limits.upper is not None
            and joint.motion_limits.upper > 0.0
            for joint in prismatic_joints
        ),
        details="Every telescoping section should translate upward on a +Z prismatic joint.",
    )
    ctx.check(
        "pan head revolves about mast centerline",
        stage_3_to_pan_head.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(value, 6) for value in stage_3_to_pan_head.axis) == (0.0, 0.0, 1.0),
        details="The pan head should rotate around the mast centerline at the top stage.",
    )

    def z_position(part) -> float:
        pos = ctx.part_world_position(part)
        return 0.0 if pos is None else float(pos[2])

    stage_1_closed_z = z_position(mast_stage_1)
    with ctx.pose({base_to_stage_1: STAGE1_TRAVEL}):
        stage_1_open_z = z_position(mast_stage_1)
    ctx.check(
        "lower mast extends upward",
        stage_1_open_z > stage_1_closed_z + 0.15,
        details=f"Expected >0.15 m vertical lift, got {stage_1_open_z - stage_1_closed_z:.3f} m.",
    )

    stage_2_closed_z = z_position(mast_stage_2)
    with ctx.pose({stage_1_to_stage_2: STAGE2_TRAVEL}):
        stage_2_open_z = z_position(mast_stage_2)
    ctx.check(
        "middle mast extends upward",
        stage_2_open_z > stage_2_closed_z + 0.12,
        details=f"Expected >0.12 m vertical lift, got {stage_2_open_z - stage_2_closed_z:.3f} m.",
    )

    stage_3_closed_z = z_position(mast_stage_3)
    with ctx.pose({stage_2_to_stage_3: STAGE3_TRAVEL}):
        stage_3_open_z = z_position(mast_stage_3)
    ctx.check(
        "upper mast extends upward",
        stage_3_open_z > stage_3_closed_z + 0.10,
        details=f"Expected >0.10 m vertical lift, got {stage_3_open_z - stage_3_closed_z:.3f} m.",
    )

    def aabb_extents(aabb) -> tuple[float, float, float]:
        return (
            float(aabb[1][0] - aabb[0][0]),
            float(aabb[1][1] - aabb[0][1]),
            float(aabb[1][2] - aabb[0][2]),
        )

    with ctx.pose({stage_3_to_pan_head: 0.0}):
        faceplate_aabb_0 = ctx.part_element_world_aabb(pan_head, elem="faceplate")
    with ctx.pose({stage_3_to_pan_head: pi * 0.5}):
        faceplate_aabb_90 = ctx.part_element_world_aabb(pan_head, elem="faceplate")

    faceplate_rotates = False
    if faceplate_aabb_0 is not None and faceplate_aabb_90 is not None:
        dx_0, dy_0, _ = aabb_extents(faceplate_aabb_0)
        dx_90, dy_90, _ = aabb_extents(faceplate_aabb_90)
        faceplate_rotates = dx_0 < 0.03 and dy_0 > 0.08 and dx_90 > 0.08 and dy_90 < 0.03
    ctx.check(
        "faceplate footprint turns with pan head",
        faceplate_rotates,
        details="The compact faceplate should visibly rotate in plan view when the pan joint turns.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
