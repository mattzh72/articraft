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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


POST_SIZE = 0.14
POST_HEIGHT = 1.82
POST_CAP_HEIGHT = 0.03
POST_FRONT_Y = POST_SIZE * 0.5

HINGE_AXIS_X = 0.012
HINGE_AXIS_Y = 0.061
GATE_BOTTOM_Z = 0.09

LEAF_WIDTH = 1.00
LEAF_DEPTH = 0.038
LEAF_CENTER_Y = -0.010
LEAF_FRONT_Y = LEAF_CENTER_Y + LEAF_DEPTH * 0.5

STILE_WIDTH = 0.09
BOTTOM_RAIL_HEIGHT = 0.12
TOP_RAIL_LEFT_X = 0.04
TOP_RAIL_WIDTH = 0.92
TOP_RAIL_BOTTOM_Z = 1.16
TOP_RAIL_END_Z = 1.29
TOP_RAIL_CENTER_Z = 1.46

RIGHT_POST_INNER_X = HINGE_AXIS_X + LEAF_WIDTH + 0.025
RIGHT_POST_CENTER_X = RIGHT_POST_INNER_X + POST_SIZE * 0.5


def _arch_profile(
    *,
    length: float,
    bottom_z: float,
    end_top_z: float,
    center_top_z: float,
    samples: int = 18,
) -> list[tuple[float, float]]:
    top_points: list[tuple[float, float]] = []
    half = length * 0.5
    rise = center_top_z - end_top_z
    for index in range(samples + 1):
        t = index / samples
        x = length * (1.0 - t)
        normalized = (x - half) / half if half > 1e-9 else 0.0
        z = end_top_z + rise * (1.0 - normalized * normalized)
        top_points.append((x, z))
    return [(0.0, bottom_z), (length, bottom_z), *top_points]


def _top_rail_mesh() -> object:
    profile = _arch_profile(
        length=TOP_RAIL_WIDTH,
        bottom_z=TOP_RAIL_BOTTOM_Z,
        end_top_z=TOP_RAIL_END_Z,
        center_top_z=TOP_RAIL_CENTER_Z,
    )
    geom = ExtrudeGeometry(profile, LEAF_DEPTH, center=True).rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, "cedar_gate_top_rail")


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[index] + upper[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cedar_garden_gate")

    cedar = model.material("cedar", rgba=(0.62, 0.41, 0.23, 1.0))
    cedar_dark = model.material("cedar_dark", rgba=(0.49, 0.32, 0.17, 1.0))
    iron = model.material("black_iron", rgba=(0.17, 0.18, 0.18, 1.0))
    steel = model.material("steel", rgba=(0.50, 0.52, 0.54, 1.0))

    posts_frame = model.part("posts_frame")
    posts_frame.visual(
        Box((POST_SIZE, POST_SIZE, POST_HEIGHT)),
        origin=Origin(
            xyz=(
                -POST_SIZE * 0.5,
                0.0,
                POST_HEIGHT * 0.5,
            )
        ),
        material=cedar_dark,
        name="left_post",
    )
    posts_frame.visual(
        Box((POST_SIZE, POST_SIZE, POST_HEIGHT)),
        origin=Origin(
            xyz=(
                RIGHT_POST_CENTER_X,
                0.0,
                POST_HEIGHT * 0.5,
            )
        ),
        material=cedar_dark,
        name="right_post",
    )
    for name, center_x in (("left_cap", -POST_SIZE * 0.5), ("right_cap", RIGHT_POST_CENTER_X)):
        posts_frame.visual(
            Box((0.16, 0.16, POST_CAP_HEIGHT)),
            origin=Origin(xyz=(center_x, 0.0, POST_HEIGHT + POST_CAP_HEIGHT * 0.5)),
            material=cedar,
            name=name,
        )

    tie_center_x = (RIGHT_POST_CENTER_X - POST_SIZE * 0.5) * 0.5
    tie_length = RIGHT_POST_CENTER_X + POST_SIZE * 0.5
    posts_frame.visual(
        Box((tie_length, 0.10, 0.16)),
        origin=Origin(xyz=(tie_center_x, 0.0, -0.08)),
        material=cedar_dark,
        name="subgrade_tie",
    )

    hinge_plate_x = -0.015
    hinge_plate_y = POST_FRONT_Y + 0.0025
    for index, z_center in enumerate((0.43, 1.06)):
        posts_frame.visual(
            Box((0.030, 0.005, 0.18)),
            origin=Origin(xyz=(hinge_plate_x, hinge_plate_y, z_center)),
            material=iron,
            name=f"post_hinge_plate_{index}",
        )

    posts_frame.visual(
        Box((0.012, 0.010, 0.060)),
        origin=Origin(
            xyz=(
                RIGHT_POST_INNER_X - 0.006,
                POST_FRONT_Y + 0.005,
                0.94,
            )
        ),
        material=iron,
        name="keeper",
    )
    posts_frame.visual(
        Box((0.028, 0.006, 0.120)),
        origin=Origin(
            xyz=(
                RIGHT_POST_INNER_X - 0.014,
                POST_FRONT_Y + 0.003,
                0.92,
            )
        ),
        material=iron,
        name="keeper_backplate",
    )
    posts_frame.inertial = Inertial.from_geometry(
        Box((RIGHT_POST_CENTER_X + 0.14, POST_SIZE, POST_HEIGHT + 0.12)),
        mass=48.0,
        origin=Origin(
            xyz=(
                tie_center_x,
                0.0,
                (POST_HEIGHT + 0.12) * 0.5 - 0.02,
            )
        ),
    )

    gate_leaf = model.part("gate_leaf")
    gate_leaf.visual(
        Box((STILE_WIDTH, LEAF_DEPTH, 1.29)),
        origin=Origin(xyz=(0.045, LEAF_CENTER_Y, 0.645)),
        material=cedar,
        name="hinge_stile",
    )
    gate_leaf.visual(
        Box((STILE_WIDTH, LEAF_DEPTH, 1.29)),
        origin=Origin(xyz=(0.955, LEAF_CENTER_Y, 0.645)),
        material=cedar,
        name="free_stile",
    )
    gate_leaf.visual(
        Box((0.92, LEAF_DEPTH, BOTTOM_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.50, LEAF_CENTER_Y, BOTTOM_RAIL_HEIGHT * 0.5)),
        material=cedar_dark,
        name="bottom_rail",
    )
    gate_leaf.visual(
        _top_rail_mesh(),
        origin=Origin(xyz=(TOP_RAIL_LEFT_X, LEAF_CENTER_Y, 0.0)),
        material=cedar_dark,
        name="top_rail",
    )

    picket_centers = [0.145 + 0.071 * index for index in range(11)]
    for index, center_x in enumerate(picket_centers):
        gate_leaf.visual(
            Box((0.052, 0.018, 1.10)),
            origin=Origin(xyz=(center_x, -0.004, 0.63)),
            material=cedar,
            name=f"picket_{index}",
        )

    for index, z_center in enumerate((0.43, 1.06)):
        gate_leaf.visual(
            Cylinder(radius=0.0105, length=0.18),
            origin=Origin(xyz=(0.0, 0.0, z_center)),
            material=iron,
            name=f"hinge_barrel_{index}",
        )
        gate_leaf.visual(
            Box((0.26, 0.005, 0.060)),
            origin=Origin(xyz=(0.145, LEAF_FRONT_Y + 0.0025, z_center)),
            material=iron,
            name=f"strap_hinge_{index}",
        )
        gate_leaf.visual(
            Box((0.035, 0.005, 0.18)),
            origin=Origin(xyz=(0.0175, LEAF_FRONT_Y + 0.0025, z_center)),
            material=iron,
            name=f"hinge_mount_{index}",
        )

    gate_leaf.inertial = Inertial.from_geometry(
        Box((LEAF_WIDTH, 0.08, TOP_RAIL_CENTER_Z)),
        mass=18.0,
        origin=Origin(xyz=(0.50, LEAF_CENTER_Y, TOP_RAIL_CENTER_Z * 0.5)),
    )

    model.articulation(
        "post_to_gate",
        ArticulationType.REVOLUTE,
        parent=posts_frame,
        child=gate_leaf,
        origin=Origin(xyz=(HINGE_AXIS_X, HINGE_AXIS_Y, GATE_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )

    thumb_latch = model.part("thumb_latch")
    thumb_latch.visual(
        Box((0.026, 0.006, 0.050)),
        origin=Origin(xyz=(0.013, 0.003, 0.0)),
        material=iron,
        name="pivot_bracket",
    )
    thumb_latch.visual(
        Cylinder(radius=0.007, length=0.012),
        origin=Origin(xyz=(0.010, 0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_hub",
    )
    thumb_latch.visual(
        Box((0.020, 0.006, 0.090)),
        origin=Origin(xyz=(0.012, 0.009, -0.050)),
        material=iron,
        name="thumb_tab",
    )
    thumb_latch.visual(
        Box((0.080, 0.006, 0.012)),
        origin=Origin(xyz=(0.044, 0.009, 0.022)),
        material=iron,
        name="latch_bar",
    )
    thumb_latch.visual(
        Box((0.012, 0.008, 0.024)),
        origin=Origin(xyz=(0.090, 0.010, 0.020)),
        material=iron,
        name="latch_tip",
    )
    thumb_latch.inertial = Inertial.from_geometry(
        Box((0.11, 0.02, 0.12)),
        mass=0.35,
        origin=Origin(xyz=(0.048, 0.010, -0.010)),
    )

    model.articulation(
        "gate_to_latch",
        ArticulationType.REVOLUTE,
        parent=gate_leaf,
        child=thumb_latch,
        origin=Origin(xyz=(0.908, LEAF_FRONT_Y, 0.92)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(35.0),
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

    posts_frame = object_model.get_part("posts_frame")
    gate_leaf = object_model.get_part("gate_leaf")
    thumb_latch = object_model.get_part("thumb_latch")
    gate_hinge = object_model.get_articulation("post_to_gate")
    latch_joint = object_model.get_articulation("gate_to_latch")

    ctx.check(
        "gate hinge uses vertical axis",
        tuple(round(value, 6) for value in gate_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"axis={gate_hinge.axis}",
    )
    ctx.check(
        "thumb latch uses local through-thickness pivot",
        tuple(round(value, 6) for value in latch_joint.axis) == (0.0, -1.0, 0.0),
        details=f"axis={latch_joint.axis}",
    )

    with ctx.pose({gate_hinge: 0.0, latch_joint: 0.0}):
        ctx.expect_gap(
            posts_frame,
            gate_leaf,
            axis="x",
            positive_elem="right_post",
            negative_elem="free_stile",
            min_gap=0.015,
            max_gap=0.040,
            name="closed leaf clears latch post",
        )
        ctx.expect_contact(
            thumb_latch,
            gate_leaf,
            elem_a="pivot_bracket",
            elem_b="free_stile",
            name="latch bracket mounts to free stile",
        )
        ctx.expect_gap(
            posts_frame,
            thumb_latch,
            axis="x",
            positive_elem="keeper",
            negative_elem="latch_tip",
            min_gap=0.002,
            max_gap=0.015,
            name="latch tip reaches keeper zone",
        )

    free_stile_closed = _aabb_center(ctx.part_element_world_aabb(gate_leaf, elem="free_stile"))
    with ctx.pose({gate_hinge: math.radians(100.0), latch_joint: 0.0}):
        free_stile_open = _aabb_center(ctx.part_element_world_aabb(gate_leaf, elem="free_stile"))
    ctx.check(
        "gate opens outward from hinge post",
        free_stile_closed is not None
        and free_stile_open is not None
        and free_stile_open[1] > free_stile_closed[1] + 0.55
        and free_stile_open[0] < free_stile_closed[0] - 0.45,
        details=f"closed={free_stile_closed}, open={free_stile_open}",
    )

    with ctx.pose({gate_hinge: 0.0, latch_joint: 0.0}):
        thumb_tab_rest = _aabb_center(ctx.part_element_world_aabb(thumb_latch, elem="thumb_tab"))
    with ctx.pose({gate_hinge: 0.0, latch_joint: math.radians(30.0)}):
        thumb_tab_pressed = _aabb_center(ctx.part_element_world_aabb(thumb_latch, elem="thumb_tab"))
    ctx.check(
        "thumb latch rotates under positive motion",
        thumb_tab_rest is not None
        and thumb_tab_pressed is not None
        and math.dist(thumb_tab_rest, thumb_tab_pressed) > 0.02,
        details=f"rest={thumb_tab_rest}, pressed={thumb_tab_pressed}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
