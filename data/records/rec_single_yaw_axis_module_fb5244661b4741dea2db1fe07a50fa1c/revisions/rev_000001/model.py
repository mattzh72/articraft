from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_RADIUS = 0.090
BASE_THICKNESS = 0.014
BEARING_RADIUS = 0.028
BEARING_HEIGHT = 0.012
BEARING_INTERLOCK = 0.0004
DECK_RADIUS = 0.075
DECK_THICKNESS = 0.008
DECK_CAP_RADIUS = 0.021
DECK_CAP_THICKNESS = 0.0012
DECK_RUNNING_GAP = 0.0

BEARING_TOP_Z = BASE_THICKNESS - BEARING_INTERLOCK + BEARING_HEIGHT
DECK_BOTTOM_Z = BEARING_TOP_Z + DECK_RUNNING_GAP
DECK_TOP_Z = DECK_BOTTOM_Z + DECK_THICKNESS + DECK_CAP_THICKNESS


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_wafer_yaw_module")

    model.material("base_black", rgba=(0.17, 0.18, 0.20, 1.0))
    model.material("bearing_steel", rgba=(0.62, 0.64, 0.67, 1.0))
    model.material("deck_aluminum", rgba=(0.74, 0.76, 0.79, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material="base_black",
        name="base_flange",
    )
    base.visual(
        Cylinder(radius=BEARING_RADIUS, length=BEARING_HEIGHT),
        origin=Origin(
            xyz=(0.0, 0.0, BASE_THICKNESS - BEARING_INTERLOCK + BEARING_HEIGHT / 2.0)
        ),
        material="bearing_steel",
        name="bearing_body",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_RADIUS, length=BEARING_TOP_Z),
        mass=1.35,
        origin=Origin(xyz=(0.0, 0.0, BEARING_TOP_Z / 2.0)),
    )

    upper_deck = model.part("upper_deck")
    upper_deck.visual(
        Cylinder(radius=DECK_RADIUS, length=DECK_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, DECK_THICKNESS / 2.0)),
        material="deck_aluminum",
        name="deck_plate",
    )
    upper_deck.visual(
        Cylinder(radius=DECK_CAP_RADIUS, length=DECK_CAP_THICKNESS),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                DECK_THICKNESS + DECK_CAP_THICKNESS / 2.0,
            )
        ),
        material="bearing_steel",
        name="deck_cap",
    )
    upper_deck.inertial = Inertial.from_geometry(
        Cylinder(radius=DECK_RADIUS, length=DECK_THICKNESS + DECK_CAP_THICKNESS),
        mass=0.62,
        origin=Origin(
            xyz=(0.0, 0.0, (DECK_THICKNESS + DECK_CAP_THICKNESS) / 2.0)
        ),
    )

    model.articulation(
        "base_to_upper_deck",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_deck,
        origin=Origin(xyz=(0.0, 0.0, DECK_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=3.5,
            lower=-math.pi,
            upper=math.pi,
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

    ctx.check(
        "yaw articulation is a vertical revolute joint",
        yaw.articulation_type == ArticulationType.REVOLUTE
        and yaw.axis == (0.0, 0.0, 1.0),
        details=(
            f"type={yaw.articulation_type}, axis={yaw.axis}, "
            f"origin={yaw.origin.xyz if yaw.origin else None}"
        ),
    )
    ctx.expect_origin_distance(
        upper_deck,
        base,
        axes="xy",
        min_dist=0.0,
        max_dist=1e-6,
        name="upper deck stays centered on the base centerline",
    )
    ctx.expect_contact(
        upper_deck,
        base,
        elem_a="deck_plate",
        elem_b="bearing_body",
        contact_tol=1e-6,
        name="upper deck seats directly on the bearing body",
    )
    ctx.expect_gap(
        upper_deck,
        base,
        axis="z",
        positive_elem="deck_plate",
        negative_elem="base_flange",
        min_gap=0.011,
        max_gap=0.013,
        name="bearing body stays visibly exposed beneath the deck",
    )
    ctx.expect_overlap(
        upper_deck,
        base,
        axes="xy",
        elem_a="deck_plate",
        elem_b="base_flange",
        min_overlap=0.145,
        name="upper deck remains broadly supported over the circular base",
    )

    with ctx.pose({yaw: 1.2}):
        ctx.expect_contact(
            upper_deck,
            base,
            elem_a="deck_plate",
            elem_b="bearing_body",
            contact_tol=1e-6,
            name="rotated deck stays seated on the bearing body",
        )
        ctx.expect_overlap(
            upper_deck,
            base,
            axes="xy",
            elem_a="deck_plate",
            elem_b="base_flange",
            min_overlap=0.145,
            name="rotated deck stays centered over the base footprint",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
