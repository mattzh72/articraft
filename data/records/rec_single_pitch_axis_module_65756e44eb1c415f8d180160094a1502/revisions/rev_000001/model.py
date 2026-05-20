from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LEN = 0.150
BASE_WIDTH = 0.120
BASE_THICK = 0.012
MOUNT_HOLE_D = 0.007

CHEEK_THICK = 0.014
CHEEK_CENTER_Y = 0.042
CHEEK_WEB_LEN = 0.060
CHEEK_WEB_HEIGHT = 0.052
CHEEK_WEB_CENTER_X = -0.012
AXIS_Z = 0.070
CHEEK_BOSS_R = 0.020
BORE_R = 0.0115

HEAD_FRONT_LEN = 0.044
HEAD_FRONT_WIDTH = 0.060
HEAD_FRONT_HEIGHT = 0.056
HEAD_FRONT_CENTER_X = 0.020

HEAD_REAR_LEN = 0.018
HEAD_REAR_WIDTH = 0.046
HEAD_REAR_HEIGHT = 0.040
HEAD_REAR_CENTER_X = -0.017

TRUNNION_BOSS_R = 0.015
TRUNNION_BOSS_LEN = 0.070


def _build_base_plate() -> cq.Workplane:
    mount_points = [
        (-0.050, -0.037),
        (-0.050, 0.037),
        (0.050, -0.037),
        (0.050, 0.037),
    ]
    return (
        cq.Workplane("XY")
        .box(BASE_LEN, BASE_WIDTH, BASE_THICK, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.005)
        .faces(">Z")
        .workplane()
        .pushPoints(mount_points)
        .hole(MOUNT_HOLE_D)
    )


def _build_single_cheek(y_center: float) -> cq.Workplane:
    web = (
        cq.Workplane("XY")
        .box(
            CHEEK_WEB_LEN,
            CHEEK_THICK,
            CHEEK_WEB_HEIGHT,
            centered=(True, True, False),
        )
        .translate((CHEEK_WEB_CENTER_X, y_center, BASE_THICK))
    )

    boss = (
        cq.Workplane("XZ")
        .circle(CHEEK_BOSS_R)
        .extrude(CHEEK_THICK / 2.0, both=True)
        .translate((0.0, y_center, AXIS_Z))
    )

    gusset = (
        cq.Workplane("XZ")
        .moveTo(-0.042, BASE_THICK)
        .lineTo(-0.042, BASE_THICK + 0.010)
        .lineTo(-0.008, AXIS_Z - 0.008)
        .lineTo(0.004, AXIS_Z - 0.008)
        .lineTo(0.004, BASE_THICK)
        .close()
        .extrude(CHEEK_THICK / 2.0, both=True)
        .translate((0.0, y_center, 0.0))
    )

    bore = (
        cq.Workplane("XZ")
        .circle(BORE_R)
        .extrude((CHEEK_THICK + 0.004) / 2.0, both=True)
        .translate((0.0, y_center, AXIS_Z))
    )

    return web.union(boss).union(gusset).cut(bore)


def _build_support_cheeks() -> cq.Workplane:
    left = _build_single_cheek(-CHEEK_CENTER_Y)
    right = _build_single_cheek(CHEEK_CENTER_Y)
    return left.union(right)


def _build_moving_head() -> cq.Workplane:
    front_body = (
        cq.Workplane("XY")
        .box(HEAD_FRONT_LEN, HEAD_FRONT_WIDTH, HEAD_FRONT_HEIGHT)
        .translate((HEAD_FRONT_CENTER_X, 0.0, 0.0))
        .edges("|Y")
        .fillet(0.006)
    )

    rear_body = (
        cq.Workplane("XY")
        .box(HEAD_REAR_LEN, HEAD_REAR_WIDTH, HEAD_REAR_HEIGHT)
        .translate((HEAD_REAR_CENTER_X, 0.0, 0.0))
        .edges("|Y")
        .fillet(0.004)
    )

    trunnion_boss = (
        cq.Workplane("XZ")
        .circle(TRUNNION_BOSS_R)
        .extrude(TRUNNION_BOSS_LEN / 2.0, both=True)
    )

    return front_body.union(rear_body).union(trunnion_boss)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="split_cheek_trunnion_module")

    model.material("machined_aluminum", rgba=(0.77, 0.79, 0.82, 1.0))
    model.material("anodized_graphite", rgba=(0.22, 0.24, 0.27, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_plate(), "base_plate"),
        material="machined_aluminum",
        name="base_plate",
    )
    base.visual(
        mesh_from_cadquery(_build_support_cheeks(), "support_cheeks"),
        material="machined_aluminum",
        name="support_cheeks",
    )

    moving_head = model.part("moving_head")
    moving_head.visual(
        mesh_from_cadquery(_build_moving_head(), "moving_head"),
        material="anodized_graphite",
        name="head_body",
    )

    model.articulation(
        "pitch_trunnion",
        ArticulationType.REVOLUTE,
        parent=base,
        child=moving_head,
        origin=Origin(xyz=(0.0, 0.0, AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.55,
            upper=0.95,
            effort=12.0,
            velocity=1.5,
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
    base = object_model.get_part("base")
    moving_head = object_model.get_part("moving_head")
    pitch = object_model.get_articulation("pitch_trunnion")

    ctx.expect_origin_gap(
        moving_head,
        base,
        axis="z",
        min_gap=AXIS_Z - 0.0005,
        max_gap=AXIS_Z + 0.0005,
        name="trunnion axis sits at the cheek bore height",
    )
    ctx.expect_within(
        moving_head,
        base,
        axes="y",
        inner_elem="head_body",
        outer_elem="support_cheeks",
        margin=0.0,
        name="moving head stays within the side-cheek span",
    )
    ctx.expect_gap(
        moving_head,
        base,
        axis="z",
        positive_elem="head_body",
        negative_elem="base_plate",
        min_gap=0.010,
        name="moving head clears the grounded base plate",
    )
    ctx.expect_contact(
        moving_head,
        base,
        elem_a="head_body",
        elem_b="support_cheeks",
        name="trunnion shoulders seat against the split cheeks",
    )

    def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_aabb = ctx.part_element_world_aabb(moving_head, elem="head_body")
    rest_center = _aabb_center(rest_aabb)
    with ctx.pose({pitch: 0.75}):
        upper_aabb = ctx.part_element_world_aabb(moving_head, elem="head_body")
        upper_center = _aabb_center(upper_aabb)
    with ctx.pose({pitch: -0.45}):
        lower_aabb = ctx.part_element_world_aabb(moving_head, elem="head_body")
        lower_center = _aabb_center(lower_aabb)
        ctx.expect_gap(
            moving_head,
            base,
            axis="z",
            positive_elem="head_body",
            negative_elem="base_plate",
            min_gap=0.010,
            name="down-pitched head still clears the base plate",
        )

    ctx.check(
        "positive pitch raises the output face",
        rest_center is not None
        and upper_center is not None
        and upper_center[2] > rest_center[2] + 0.004,
        details=f"rest_center={rest_center}, upper_center={upper_center}",
    )
    ctx.check(
        "negative pitch lowers the output face",
        rest_center is not None
        and lower_center is not None
        and lower_center[2] < rest_center[2] - 0.002,
        details=f"rest_center={rest_center}, lower_center={lower_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
