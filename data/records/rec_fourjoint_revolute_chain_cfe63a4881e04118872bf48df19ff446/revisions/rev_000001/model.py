from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


LINK_SPAN = 0.145
BLOCK_LEN = 0.028
BLOCK_HEIGHT = 0.055
OUTER_WIDTH = 0.028
CENTER_BLOCK_WIDTH = 0.011
RAIL_HEIGHT = 0.010
RAIL_OFFSET_Z = 0.018
RUNG_LEN = 0.012
RUNG_HEIGHT = 2.0 * RAIL_OFFSET_Z - RAIL_HEIGHT

BASE_PLATE_THICKNESS = 0.016
BASE_PLATE_WIDTH = 0.044
BASE_PLATE_HEIGHT = 0.096

OUTPUT_PLATE_LEN = 0.050
OUTPUT_PLATE_HEIGHT = 0.060

HINGE_LIMITS = (0.0, 1.05)


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _moving_link_shape() -> cq.Workplane:
    rail_len = LINK_SPAN - 2.0 * BLOCK_LEN

    link = _box((BLOCK_LEN, CENTER_BLOCK_WIDTH, BLOCK_HEIGHT), (BLOCK_LEN / 2.0, 0.0, 0.0))
    link = link.union(
        _box(
            (rail_len, OUTER_WIDTH, RAIL_HEIGHT),
            (BLOCK_LEN + rail_len / 2.0, 0.0, RAIL_OFFSET_Z),
        )
    )
    link = link.union(
        _box(
            (rail_len, OUTER_WIDTH, RAIL_HEIGHT),
            (BLOCK_LEN + rail_len / 2.0, 0.0, -RAIL_OFFSET_Z),
        )
    )
    link = link.union(
        _box(
            (RUNG_LEN, OUTER_WIDTH, RUNG_HEIGHT),
            (LINK_SPAN * 0.54, 0.0, 0.0),
        )
    )
    link = link.union(
        _box(
            (BLOCK_LEN, OUTER_WIDTH, BLOCK_HEIGHT),
            (LINK_SPAN - BLOCK_LEN / 2.0, 0.0, 0.0),
        )
    )
    return link


def _base_lug_shape() -> cq.Workplane:
    rail_len = LINK_SPAN - BLOCK_LEN

    base = _box(
        (BASE_PLATE_THICKNESS, BASE_PLATE_WIDTH, BASE_PLATE_HEIGHT),
        (-BASE_PLATE_THICKNESS / 2.0, 0.0, 0.0),
    )
    base = base.union(
        _box(
            (rail_len, OUTER_WIDTH, RAIL_HEIGHT),
            (rail_len / 2.0, 0.0, RAIL_OFFSET_Z),
        )
    )
    base = base.union(
        _box(
            (rail_len, OUTER_WIDTH, RAIL_HEIGHT),
            (rail_len / 2.0, 0.0, -RAIL_OFFSET_Z),
        )
    )
    base = base.union(
        _box(
            (RUNG_LEN, OUTER_WIDTH, RUNG_HEIGHT),
            (LINK_SPAN * 0.45, 0.0, 0.0),
        )
    )
    base = base.union(
        _box(
            (BASE_PLATE_THICKNESS, BASE_PLATE_WIDTH, 0.030),
            (BASE_PLATE_THICKNESS / 2.0, 0.0, 0.0),
        )
    )
    base = base.union(
        _box(
            (BLOCK_LEN, OUTER_WIDTH, BLOCK_HEIGHT),
            (LINK_SPAN - BLOCK_LEN / 2.0, 0.0, 0.0),
        )
    )
    return base


def _output_link_shape() -> cq.Workplane:
    rail_len = LINK_SPAN - BLOCK_LEN + OUTPUT_PLATE_LEN * 0.35
    plate_center_x = LINK_SPAN + OUTPUT_PLATE_LEN / 2.0

    link = _box((BLOCK_LEN, CENTER_BLOCK_WIDTH, BLOCK_HEIGHT), (BLOCK_LEN / 2.0, 0.0, 0.0))
    link = link.union(
        _box(
            (rail_len, OUTER_WIDTH, RAIL_HEIGHT),
            (BLOCK_LEN + rail_len / 2.0, 0.0, RAIL_OFFSET_Z),
        )
    )
    link = link.union(
        _box(
            (rail_len, OUTER_WIDTH, RAIL_HEIGHT),
            (BLOCK_LEN + rail_len / 2.0, 0.0, -RAIL_OFFSET_Z),
        )
    )
    link = link.union(
        _box(
            (RUNG_LEN, OUTER_WIDTH, RUNG_HEIGHT),
            (LINK_SPAN * 0.52, 0.0, 0.0),
        )
    )
    link = link.union(
        _box(
            (OUTPUT_PLATE_LEN, OUTER_WIDTH, OUTPUT_PLATE_HEIGHT),
            (plate_center_x, 0.0, 0.0),
        )
    )
    return link


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ladder_frame_hinge_chain")

    model.material("graphite", rgba=(0.19, 0.20, 0.22, 1.0))
    model.material("link_steel", rgba=(0.63, 0.66, 0.70, 1.0))
    model.material("end_plate_finish", rgba=(0.77, 0.52, 0.18, 1.0))

    base_lug = model.part("base_lug")
    base_lug.visual(
        mesh_from_cadquery(_base_lug_shape(), "base_lug"),
        material="graphite",
        name="base_member",
    )
    base_lug.inertial = Inertial.from_geometry(
        Box((LINK_SPAN + BASE_PLATE_THICKNESS, BASE_PLATE_WIDTH, BASE_PLATE_HEIGHT)),
        mass=1.2,
        origin=Origin(xyz=(LINK_SPAN / 2.0 - BASE_PLATE_THICKNESS / 2.0, 0.0, 0.0)),
    )

    link_1 = model.part("link_1")
    link_1.visual(
        mesh_from_cadquery(_moving_link_shape(), "link_1"),
        material="link_steel",
        name="link_1_member",
    )
    link_1.inertial = Inertial.from_geometry(
        Box((LINK_SPAN + BLOCK_LEN, OUTER_WIDTH, BLOCK_HEIGHT)),
        mass=0.32,
        origin=Origin(xyz=(LINK_SPAN / 2.0, 0.0, 0.0)),
    )

    link_2 = model.part("link_2")
    link_2.visual(
        mesh_from_cadquery(_moving_link_shape(), "link_2"),
        material="link_steel",
        name="link_2_member",
    )
    link_2.inertial = Inertial.from_geometry(
        Box((LINK_SPAN + BLOCK_LEN, OUTER_WIDTH, BLOCK_HEIGHT)),
        mass=0.30,
        origin=Origin(xyz=(LINK_SPAN / 2.0, 0.0, 0.0)),
    )

    link_3 = model.part("link_3")
    link_3.visual(
        mesh_from_cadquery(_moving_link_shape(), "link_3"),
        material="link_steel",
        name="link_3_member",
    )
    link_3.inertial = Inertial.from_geometry(
        Box((LINK_SPAN + BLOCK_LEN, OUTER_WIDTH, BLOCK_HEIGHT)),
        mass=0.28,
        origin=Origin(xyz=(LINK_SPAN / 2.0, 0.0, 0.0)),
    )

    link_4 = model.part("link_4")
    link_4.visual(
        mesh_from_cadquery(_output_link_shape(), "link_4"),
        material="end_plate_finish",
        name="link_4_member",
    )
    link_4.inertial = Inertial.from_geometry(
        Box((LINK_SPAN + OUTPUT_PLATE_LEN + BLOCK_LEN, OUTER_WIDTH, OUTPUT_PLATE_HEIGHT)),
        mass=0.34,
        origin=Origin(xyz=((LINK_SPAN + OUTPUT_PLATE_LEN) / 2.0, 0.0, 0.0)),
    )

    hinge_kwargs = dict(
        articulation_type=ArticulationType.REVOLUTE,
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=HINGE_LIMITS[0],
            upper=HINGE_LIMITS[1],
        ),
    )

    model.articulation(
        "hinge_1",
        parent=base_lug,
        child=link_1,
        origin=Origin(xyz=(LINK_SPAN, 0.0, 0.0)),
        **hinge_kwargs,
    )
    model.articulation(
        "hinge_2",
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(LINK_SPAN, 0.0, 0.0)),
        **hinge_kwargs,
    )
    model.articulation(
        "hinge_3",
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(LINK_SPAN, 0.0, 0.0)),
        **hinge_kwargs,
    )
    model.articulation(
        "hinge_4",
        parent=link_3,
        child=link_4,
        origin=Origin(xyz=(LINK_SPAN, 0.0, 0.0)),
        **hinge_kwargs,
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
    base_lug = object_model.get_part("base_lug")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")
    link_4 = object_model.get_part("link_4")

    hinge_1 = object_model.get_articulation("hinge_1")
    hinge_2 = object_model.get_articulation("hinge_2")
    hinge_3 = object_model.get_articulation("hinge_3")
    hinge_4 = object_model.get_articulation("hinge_4")

    chain_parts = [base_lug, link_1, link_2, link_3, link_4]
    hinges = [hinge_1, hinge_2, hinge_3, hinge_4]

    ctx.check(
        "five visible chain members are authored",
        len(object_model.parts) == 5,
        details=f"parts={[part.name for part in object_model.parts]}",
    )
    ctx.check(
        "four serial revolute joints are authored",
        len(object_model.articulations) == 4
        and all(joint.articulation_type == ArticulationType.REVOLUTE for joint in hinges),
        details=f"articulations={[joint.name for joint in object_model.articulations]}",
    )
    ctx.check(
        "all hinge axes are planar y-axis pivots",
        all(joint.axis == (0.0, -1.0, 0.0) for joint in hinges),
        details=f"axes={[joint.axis for joint in hinges]}",
    )

    ctx.expect_overlap(
        base_lug,
        link_1,
        axes="yz",
        elem_a="base_member",
        elem_b="link_1_member",
        min_overlap=0.022,
        name="base lug and first link align across the first hinge face",
    )
    ctx.expect_gap(
        link_1,
        base_lug,
        axis="x",
        positive_elem="link_1_member",
        negative_elem="base_member",
        min_gap=0.0,
        max_gap=0.001,
        name="base lug and first link meet without overlap at the first hinge",
    )
    ctx.expect_overlap(
        link_1,
        link_2,
        axes="yz",
        elem_a="link_1_member",
        elem_b="link_2_member",
        min_overlap=0.022,
        name="first and second links align across the second hinge face",
    )
    ctx.expect_gap(
        link_2,
        link_1,
        axis="x",
        positive_elem="link_2_member",
        negative_elem="link_1_member",
        min_gap=0.0,
        max_gap=0.001,
        name="first and second links meet without overlap at the second hinge",
    )
    ctx.expect_overlap(
        link_2,
        link_3,
        axes="yz",
        elem_a="link_2_member",
        elem_b="link_3_member",
        min_overlap=0.022,
        name="second and third links align across the third hinge face",
    )
    ctx.expect_gap(
        link_3,
        link_2,
        axis="x",
        positive_elem="link_3_member",
        negative_elem="link_2_member",
        min_gap=0.0,
        max_gap=0.001,
        name="second and third links meet without overlap at the third hinge",
    )
    ctx.expect_overlap(
        link_3,
        link_4,
        axes="yz",
        elem_a="link_3_member",
        elem_b="link_4_member",
        min_overlap=0.022,
        name="third and output links align across the fourth hinge face",
    )
    ctx.expect_gap(
        link_4,
        link_3,
        axis="x",
        positive_elem="link_4_member",
        negative_elem="link_3_member",
        min_gap=0.0,
        max_gap=0.001,
        name="third and output links meet without overlap at the fourth hinge",
    )

    rest_tip_aabb = ctx.part_element_world_aabb(link_4, elem="link_4_member")
    with ctx.pose({hinge_1: 0.42, hinge_2: 0.30, hinge_3: 0.22, hinge_4: 0.18}):
        posed_tip_aabb = ctx.part_element_world_aabb(link_4, elem="link_4_member")
        ctx.check(
            "positive hinge motion lifts the output plate",
            rest_tip_aabb is not None
            and posed_tip_aabb is not None
            and posed_tip_aabb[1][2] > rest_tip_aabb[1][2] + 0.050,
            details=f"rest={rest_tip_aabb}, posed={posed_tip_aabb}",
        )
        for part in chain_parts:
            pos = ctx.part_world_position(part)
            ctx.check(
                f"{part.name} stays on the single hinge plane",
                pos is not None and abs(pos[1]) < 1e-5,
                details=f"position={pos}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
