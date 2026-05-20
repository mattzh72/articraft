from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


RAIL_LENGTH = 0.18
RAIL_WIDTH = 0.012
RAIL_HEIGHT = 0.008

CARRIAGE_LENGTH = 0.045
CARRIAGE_WIDTH = 0.032
CHANNEL_WIDTH = 0.016
CHEEK_WIDTH = (CARRIAGE_WIDTH - CHANNEL_WIDTH) / 2.0
CHEEK_HEIGHT = RAIL_HEIGHT
BRIDGE_HEIGHT = 0.006
CARRIAGE_BODY_HEIGHT = CHEEK_HEIGHT + BRIDGE_HEIGHT

PLATE_LENGTH = 0.030
PLATE_WIDTH = 0.020
PLATE_HEIGHT = 0.004

SLIDE_TRAVEL = 0.055


def _aabb_size(aabb):
    if aabb is None:
        return None
    return tuple(aabb[1][i] - aabb[0][i] for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="linear_slide_module")

    rail_finish = model.material("rail_finish", rgba=(0.66, 0.68, 0.72, 1.0))
    carriage_finish = model.material("carriage_finish", rgba=(0.17, 0.18, 0.20, 1.0))
    plate_finish = model.material("plate_finish", rgba=(0.80, 0.82, 0.85, 1.0))

    guide_rail = model.part("guide_rail")
    guide_rail.visual(
        Box((RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, RAIL_HEIGHT / 2.0)),
        material=rail_finish,
        name="rail_body",
    )
    guide_rail.inertial = Inertial.from_geometry(
        Box((RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)),
        mass=0.40,
        origin=Origin(xyz=(0.0, 0.0, RAIL_HEIGHT / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, BRIDGE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, RAIL_HEIGHT + BRIDGE_HEIGHT / 2.0)),
        material=carriage_finish,
        name="top_bridge",
    )
    carriage.visual(
        Box((CARRIAGE_LENGTH, CHEEK_WIDTH, CHEEK_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                CHANNEL_WIDTH / 2.0 + CHEEK_WIDTH / 2.0,
                CHEEK_HEIGHT / 2.0,
            )
        ),
        material=carriage_finish,
        name="left_cheek",
    )
    carriage.visual(
        Box((CARRIAGE_LENGTH, CHEEK_WIDTH, CHEEK_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -(CHANNEL_WIDTH / 2.0 + CHEEK_WIDTH / 2.0),
                CHEEK_HEIGHT / 2.0,
            )
        ),
        material=carriage_finish,
        name="right_cheek",
    )
    carriage.visual(
        Box((PLATE_LENGTH, PLATE_WIDTH, PLATE_HEIGHT)),
        origin=Origin(
            xyz=(0.0, 0.0, CARRIAGE_BODY_HEIGHT + PLATE_HEIGHT / 2.0),
        ),
        material=plate_finish,
        name="mount_plate",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_BODY_HEIGHT + PLATE_HEIGHT)),
        mass=0.22,
        origin=Origin(
            xyz=(0.0, 0.0, (CARRIAGE_BODY_HEIGHT + PLATE_HEIGHT) / 2.0),
        ),
    )

    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=guide_rail,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.25,
            lower=-SLIDE_TRAVEL,
            upper=SLIDE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    guide_rail = object_model.get_part("guide_rail")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("rail_to_carriage")
    rail_body = guide_rail.get_visual("rail_body")
    top_bridge = carriage.get_visual("top_bridge")
    mount_plate = carriage.get_visual("mount_plate")

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

    ctx.check(
        "slide_joint_is_prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"Expected PRISMATIC articulation, found {slide.articulation_type!r}.",
    )
    ctx.check(
        "slide_joint_axis_is_guide_axis",
        tuple(slide.axis) == (1.0, 0.0, 0.0),
        details=f"Expected axis (1, 0, 0), found {slide.axis!r}.",
    )

    limits = slide.motion_limits
    ctx.check(
        "slide_joint_has_realistic_travel",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < 0.0 < limits.upper
        and 0.08 <= (limits.upper - limits.lower) <= 0.14,
        details=f"Unexpected motion limits: {limits!r}.",
    )

    rail_aabb = ctx.part_element_world_aabb(guide_rail, elem="rail_body")
    bridge_aabb = ctx.part_element_world_aabb(carriage, elem="top_bridge")
    plate_aabb = ctx.part_element_world_aabb(carriage, elem="mount_plate")

    rail_size = _aabb_size(rail_aabb)
    bridge_size = _aabb_size(bridge_aabb)
    plate_size = _aabb_size(plate_aabb)

    ctx.check(
        "rail_longer_than_carriage",
        rail_size is not None and bridge_size is not None and rail_size[0] > bridge_size[0],
        details=f"Rail size {rail_size!r} should be longer than carriage size {bridge_size!r}.",
    )
    ctx.check(
        "mount_plate_smaller_than_carriage",
        plate_size is not None
        and bridge_size is not None
        and plate_size[0] < bridge_size[0]
        and plate_size[1] < bridge_size[1],
        details=f"Plate size {plate_size!r} should sit within carriage size {bridge_size!r}.",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_contact(
            carriage,
            guide_rail,
            elem_a=top_bridge,
            elem_b=rail_body,
            name="center_pose_carriage_supported_on_rail",
        )
        ctx.expect_overlap(
            carriage,
            guide_rail,
            axes="xy",
            min_overlap=0.010,
            elem_a=top_bridge,
            elem_b=rail_body,
            name="center_pose_carriage_aligned_over_rail",
        )
        ctx.expect_gap(
            carriage,
            guide_rail,
            axis="z",
            positive_elem=mount_plate,
            negative_elem=rail_body,
            min_gap=0.0055,
            max_gap=0.0065,
            name="mount_plate_sits_above_rail",
        )

    if limits is not None and limits.lower is not None and limits.upper is not None:
        for pose_name, pose_value in (("lower", limits.lower), ("upper", limits.upper)):
            with ctx.pose({slide: pose_value}):
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"rail_to_carriage_{pose_name}_no_overlap"
                )
                ctx.fail_if_isolated_parts(
                    name=f"rail_to_carriage_{pose_name}_no_floating"
                )
                ctx.expect_contact(
                    carriage,
                    guide_rail,
                    elem_a=top_bridge,
                    elem_b=rail_body,
                    name=f"rail_to_carriage_{pose_name}_contact",
                )
                ctx.expect_overlap(
                    carriage,
                    guide_rail,
                    axes="xy",
                    min_overlap=0.010,
                    elem_a=top_bridge,
                    elem_b=rail_body,
                    name=f"rail_to_carriage_{pose_name}_overlap",
                )

                pose_rail_aabb = ctx.part_world_aabb(guide_rail)
                pose_carriage_aabb = ctx.part_world_aabb(carriage)
                ctx.check(
                    f"rail_to_carriage_{pose_name}_stays_on_rail_span",
                    pose_rail_aabb is not None
                    and pose_carriage_aabb is not None
                    and pose_carriage_aabb[0][0] >= pose_rail_aabb[0][0]
                    and pose_carriage_aabb[1][0] <= pose_rail_aabb[1][0],
                    details=(
                        f"Carriage AABB {pose_carriage_aabb!r} should remain within "
                        f"rail AABB {pose_rail_aabb!r} at the {pose_name} limit."
                    ),
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
