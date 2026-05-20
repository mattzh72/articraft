from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_RADIUS = 0.170
BASE_THICKNESS = 0.020
BASE_TOP_BOSS_RADIUS = 0.030
BASE_TOP_BOSS_HEIGHT = 0.002

DECK_RADIUS = 0.145
DECK_THICKNESS = 0.006
BEARING_RADIUS = 0.046
BEARING_HEIGHT = 0.007

YAW_LIMIT = 3.0


def _make_base_shape() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .circle(BASE_RADIUS)
        .extrude(BASE_THICKNESS)
        .faces(">Z")
        .edges()
        .chamfer(0.0014)
        .faces("<Z")
        .edges()
        .chamfer(0.0010)
    )

    boss = (
        cq.Workplane("XY")
        .circle(BASE_TOP_BOSS_RADIUS)
        .extrude(BASE_TOP_BOSS_HEIGHT)
        .translate((0.0, 0.0, BASE_THICKNESS))
    )

    return base.union(boss)


def _make_upper_deck_shape() -> cq.Workplane:
    bearing = (
        cq.Workplane("XY")
        .circle(BEARING_RADIUS)
        .extrude(BEARING_HEIGHT)
        .faces("<Z")
        .edges()
        .fillet(0.0006)
    )

    deck = (
        cq.Workplane("XY")
        .circle(DECK_RADIUS)
        .extrude(DECK_THICKNESS)
        .faces(">Z")
        .edges()
        .chamfer(0.0010)
        .faces("<Z")
        .edges()
        .chamfer(0.0007)
        .translate((0.0, 0.0, BEARING_HEIGHT))
    )

    return bearing.union(deck)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_wafer_yaw_module")

    model.material("powder_charcoal", rgba=(0.17, 0.18, 0.20, 1.0))
    model.material("machined_silver", rgba=(0.74, 0.76, 0.79, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "yaw_module_base"),
        material="powder_charcoal",
        name="base_shell",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS + BASE_TOP_BOSS_HEIGHT),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, (BASE_THICKNESS + BASE_TOP_BOSS_HEIGHT) / 2.0)),
    )

    upper_deck = model.part("upper_deck")
    upper_deck.visual(
        mesh_from_cadquery(_make_upper_deck_shape(), "yaw_module_upper_deck"),
        material="machined_silver",
        name="upper_deck_shell",
    )
    upper_deck.inertial = Inertial.from_geometry(
        Cylinder(radius=DECK_RADIUS, length=BEARING_HEIGHT + DECK_THICKNESS),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, (BEARING_HEIGHT + DECK_THICKNESS) / 2.0)),
    )

    model.articulation(
        "base_to_upper_deck",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_deck,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + BASE_TOP_BOSS_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-YAW_LIMIT,
            upper=YAW_LIMIT,
            effort=18.0,
            velocity=2.5,
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
    upper_deck = object_model.get_part("upper_deck")
    yaw = object_model.get_articulation("base_to_upper_deck")

    ctx.expect_contact(
        upper_deck,
        base,
        contact_tol=0.001,
        name="upper deck is physically supported by the base",
    )
    ctx.expect_overlap(
        upper_deck,
        base,
        axes="xy",
        min_overlap=0.25,
        name="upper deck stays centered over the broad circular base",
    )

    rest_pos = ctx.part_world_position(upper_deck)
    with ctx.pose({yaw: 1.2}):
        turned_pos = ctx.part_world_position(upper_deck)
        ctx.expect_contact(
            upper_deck,
            base,
            contact_tol=0.001,
            name="upper deck remains seated on the base while yawed",
        )

    ctx.check(
        "yaw joint rotates about the vertical centerline",
        yaw.axis == (0.0, 0.0, 1.0)
        and rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0] - turned_pos[0]) < 1e-6
        and abs(rest_pos[1] - turned_pos[1]) < 1e-6
        and abs(rest_pos[2] - turned_pos[2]) < 1e-6,
        details=f"axis={yaw.axis}, rest={rest_pos}, turned={turned_pos}",
    )

    upper_aabb = ctx.part_world_aabb(upper_deck)
    total_height = None if upper_aabb is None else upper_aabb[1][2] - upper_aabb[0][2]
    ctx.check(
        "upper deck remains a low-profile deck with a visible bearing body",
        total_height is not None and 0.011 <= total_height <= 0.016,
        details=f"upper deck z height={total_height}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
