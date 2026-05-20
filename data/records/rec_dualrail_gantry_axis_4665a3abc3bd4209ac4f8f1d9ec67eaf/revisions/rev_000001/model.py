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


BASE_LENGTH = 1.55
BASE_BEAM_WIDTH = 0.14
BASE_BEAM_HEIGHT = 0.08
RAIL_CENTER_Y = 0.42
BASE_TIE_WIDTH = 0.14
BASE_TIE_X = 0.68
GUIDE_LENGTH = 1.45
GUIDE_WIDTH = 0.04
GUIDE_HEIGHT = 0.03

BRIDGE_ORIGIN_Z = 0.40
BRIDGE_DEPTH = 0.16
BRIDGE_LENGTH = 1.04
BRIDGE_HEIGHT = 0.24
END_PLATE_DEPTH = 0.14
END_PLATE_WIDTH = 0.09
END_PLATE_HEIGHT = 0.29
END_PLATE_LOCAL_Z = -0.145
FRONT_GUIDE_THICKNESS = 0.022
FRONT_GUIDE_LENGTH = 0.86
FRONT_GUIDE_HEIGHT = 0.10
FRONT_GUIDE_CENTER_X = -(BRIDGE_DEPTH / 2.0) - (FRONT_GUIDE_THICKNESS / 2.0)
SADDLE_LENGTH = 0.28
SADDLE_WIDTH = 0.11
SADDLE_HEIGHT = 0.05
SADDLE_LOCAL_Z = (
    BASE_BEAM_HEIGHT + GUIDE_HEIGHT + (SADDLE_HEIGHT / 2.0) - BRIDGE_ORIGIN_Z
)

CARRIAGE_BODY_DEPTH = 0.10
CARRIAGE_BODY_WIDTH = 0.18
CARRIAGE_BODY_HEIGHT = 0.16
CARRIAGE_PLATE_THICKNESS = 0.014
CARRIAGE_PLATE_WIDTH = 0.14
CARRIAGE_PLATE_HEIGHT = 0.14
REAR_GUIDE_THICKNESS = 0.018
REAR_GUIDE_WIDTH = 0.16
REAR_GUIDE_HEIGHT = 0.12
REAR_GUIDE_LOCAL_X = 0.055
CARRIAGE_ORIGIN_X = (
    FRONT_GUIDE_CENTER_X
    - (FRONT_GUIDE_THICKNESS / 2.0)
    - (REAR_GUIDE_LOCAL_X + (REAR_GUIDE_THICKNESS / 2.0))
)

BRIDGE_TRAVEL_LOWER = -0.40
BRIDGE_TRAVEL_UPPER = 0.40
CARRIAGE_TRAVEL_LOWER = -0.34
CARRIAGE_TRAVEL_UPPER = 0.34


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).translate(center)


def _build_base_frame_shape() -> cq.Workplane:
    tie_span = (2.0 * RAIL_CENTER_Y) + BASE_BEAM_WIDTH
    frame = _box(
        (BASE_LENGTH, BASE_BEAM_WIDTH, BASE_BEAM_HEIGHT),
        (0.0, RAIL_CENTER_Y, BASE_BEAM_HEIGHT / 2.0),
    )
    frame = frame.union(
        _box(
            (BASE_LENGTH, BASE_BEAM_WIDTH, BASE_BEAM_HEIGHT),
            (0.0, -RAIL_CENTER_Y, BASE_BEAM_HEIGHT / 2.0),
        )
    )
    frame = frame.union(
        _box(
            (BASE_TIE_WIDTH, tie_span, BASE_BEAM_HEIGHT),
            (-BASE_TIE_X, 0.0, BASE_BEAM_HEIGHT / 2.0),
        )
    )
    frame = frame.union(
        _box(
            (BASE_TIE_WIDTH, tie_span, BASE_BEAM_HEIGHT),
            (BASE_TIE_X, 0.0, BASE_BEAM_HEIGHT / 2.0),
        )
    )
    return frame


def _build_bridge_frame_shape() -> cq.Workplane:
    beam = _box((BRIDGE_DEPTH, BRIDGE_LENGTH, BRIDGE_HEIGHT), (0.0, 0.0, 0.0))
    beam = beam.union(
        _box(
            (END_PLATE_DEPTH, END_PLATE_WIDTH, END_PLATE_HEIGHT),
            (0.0, RAIL_CENTER_Y, END_PLATE_LOCAL_Z),
        )
    )
    beam = beam.union(
        _box(
            (END_PLATE_DEPTH, END_PLATE_WIDTH, END_PLATE_HEIGHT),
            (0.0, -RAIL_CENTER_Y, END_PLATE_LOCAL_Z),
        )
    )
    return beam


def _build_carriage_body_shape() -> cq.Workplane:
    body = _box(
        (CARRIAGE_BODY_DEPTH, CARRIAGE_BODY_WIDTH, CARRIAGE_BODY_HEIGHT),
        (0.0, 0.0, 0.0),
    )
    body = body.union(
        _box(
            (
                CARRIAGE_PLATE_THICKNESS,
                CARRIAGE_PLATE_WIDTH,
                CARRIAGE_PLATE_HEIGHT,
            ),
            (
                -(CARRIAGE_BODY_DEPTH / 2.0) - (CARRIAGE_PLATE_THICKNESS / 2.0) + 0.004,
                0.0,
                0.0,
            ),
        )
    )
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stiff_beam_gantry_axis")

    model.material("frame_gray", rgba=(0.39, 0.42, 0.46, 1.0))
    model.material("bridge_gray", rgba=(0.73, 0.76, 0.80, 1.0))
    model.material("rail_steel", rgba=(0.56, 0.59, 0.63, 1.0))
    model.material("carriage_black", rgba=(0.16, 0.17, 0.19, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_frame_shape(), "base_frame"),
        material="frame_gray",
        name="base_frame",
    )
    base.visual(
        mesh_from_cadquery(
            _box(
                (GUIDE_LENGTH, GUIDE_WIDTH, GUIDE_HEIGHT),
                (0.0, RAIL_CENTER_Y, BASE_BEAM_HEIGHT + (GUIDE_HEIGHT / 2.0)),
            ),
            "left_guide",
        ),
        material="rail_steel",
        name="left_guide",
    )
    base.visual(
        mesh_from_cadquery(
            _box(
                (GUIDE_LENGTH, GUIDE_WIDTH, GUIDE_HEIGHT),
                (0.0, -RAIL_CENTER_Y, BASE_BEAM_HEIGHT + (GUIDE_HEIGHT / 2.0)),
            ),
            "right_guide",
        ),
        material="rail_steel",
        name="right_guide",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, (2.0 * RAIL_CENTER_Y) + BASE_BEAM_WIDTH, BASE_BEAM_HEIGHT + GUIDE_HEIGHT)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, (BASE_BEAM_HEIGHT + GUIDE_HEIGHT) / 2.0)),
    )

    bridge = model.part("bridge")
    bridge.visual(
        mesh_from_cadquery(_build_bridge_frame_shape(), "bridge_frame"),
        material="bridge_gray",
        name="bridge_frame",
    )
    bridge.visual(
        mesh_from_cadquery(
            _box(
                (FRONT_GUIDE_THICKNESS, FRONT_GUIDE_LENGTH, FRONT_GUIDE_HEIGHT),
                (FRONT_GUIDE_CENTER_X, 0.0, 0.0),
            ),
            "front_guide",
        ),
        material="rail_steel",
        name="front_guide",
    )
    bridge.visual(
        mesh_from_cadquery(
            _box(
                (SADDLE_LENGTH, SADDLE_WIDTH, SADDLE_HEIGHT),
                (0.0, RAIL_CENTER_Y, SADDLE_LOCAL_Z),
            ),
            "left_saddle",
        ),
        material="frame_gray",
        name="left_saddle",
    )
    bridge.visual(
        mesh_from_cadquery(
            _box(
                (SADDLE_LENGTH, SADDLE_WIDTH, SADDLE_HEIGHT),
                (0.0, -RAIL_CENTER_Y, SADDLE_LOCAL_Z),
            ),
            "right_saddle",
        ),
        material="frame_gray",
        name="right_saddle",
    )
    bridge.inertial = Inertial.from_geometry(
        Box((BRIDGE_DEPTH, BRIDGE_LENGTH, BRIDGE_HEIGHT)),
        mass=17.0,
        origin=Origin(),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_build_carriage_body_shape(), "carriage_body"),
        material="carriage_black",
        name="carriage_body",
    )
    carriage.visual(
        mesh_from_cadquery(
            _box(
                (REAR_GUIDE_THICKNESS, REAR_GUIDE_WIDTH, REAR_GUIDE_HEIGHT),
                (REAR_GUIDE_LOCAL_X, 0.0, 0.0),
            ),
            "rear_guide",
        ),
        material="rail_steel",
        name="rear_guide",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_BODY_DEPTH, CARRIAGE_BODY_WIDTH, CARRIAGE_BODY_HEIGHT)),
        mass=4.0,
        origin=Origin(),
    )

    model.articulation(
        "base_to_bridge",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bridge,
        origin=Origin(xyz=(0.0, 0.0, BRIDGE_ORIGIN_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=BRIDGE_TRAVEL_LOWER,
            upper=BRIDGE_TRAVEL_UPPER,
            effort=1500.0,
            velocity=0.55,
        ),
    )
    model.articulation(
        "bridge_to_carriage",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=carriage,
        origin=Origin(xyz=(CARRIAGE_ORIGIN_X, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=CARRIAGE_TRAVEL_LOWER,
            upper=CARRIAGE_TRAVEL_UPPER,
            effort=450.0,
            velocity=0.65,
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
    bridge = object_model.get_part("bridge")
    carriage = object_model.get_part("carriage")
    base_to_bridge = object_model.get_articulation("base_to_bridge")
    bridge_to_carriage = object_model.get_articulation("bridge_to_carriage")

    with ctx.pose({base_to_bridge: 0.0, bridge_to_carriage: 0.0}):
        ctx.expect_contact(
            bridge,
            base,
            elem_a="left_saddle",
            elem_b="left_guide",
            contact_tol=1e-5,
            name="left bridge saddle sits on left base guide",
        )
        ctx.expect_contact(
            bridge,
            base,
            elem_a="right_saddle",
            elem_b="right_guide",
            contact_tol=1e-5,
            name="right bridge saddle sits on right base guide",
        )
        ctx.expect_contact(
            carriage,
            bridge,
            elem_a="rear_guide",
            elem_b="front_guide",
            contact_tol=1e-5,
            name="carriage rear guide rides on bridge front guide",
        )

    with ctx.pose({base_to_bridge: BRIDGE_TRAVEL_LOWER}):
        lower_bridge_pos = ctx.part_world_position(bridge)
    with ctx.pose({base_to_bridge: BRIDGE_TRAVEL_UPPER}):
        upper_bridge_pos = ctx.part_world_position(bridge)
        ctx.expect_overlap(
            bridge,
            base,
            axes="x",
            elem_a="left_saddle",
            elem_b="left_guide",
            min_overlap=SADDLE_LENGTH - 0.01,
            name="left saddle remains inserted on the rail at max bridge travel",
        )
        ctx.expect_overlap(
            bridge,
            base,
            axes="x",
            elem_a="right_saddle",
            elem_b="right_guide",
            min_overlap=SADDLE_LENGTH - 0.01,
            name="right saddle remains inserted on the rail at max bridge travel",
        )
    ctx.check(
        "bridge translates along the base rails",
        lower_bridge_pos is not None
        and upper_bridge_pos is not None
        and upper_bridge_pos[0] > lower_bridge_pos[0] + 0.70
        and abs(upper_bridge_pos[1] - lower_bridge_pos[1]) < 1e-6
        and abs(upper_bridge_pos[2] - lower_bridge_pos[2]) < 1e-6,
        details=f"lower={lower_bridge_pos}, upper={upper_bridge_pos}",
    )

    with ctx.pose({bridge_to_carriage: CARRIAGE_TRAVEL_LOWER}):
        lower_carriage_pos = ctx.part_world_position(carriage)
    with ctx.pose({bridge_to_carriage: CARRIAGE_TRAVEL_UPPER}):
        upper_carriage_pos = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            bridge,
            axes="y",
            elem_a="rear_guide",
            elem_b="front_guide",
            min_overlap=REAR_GUIDE_WIDTH - 0.01,
            name="carriage remains captured on the bridge guide at max travel",
        )
    ctx.check(
        "carriage translates across the bridge face",
        lower_carriage_pos is not None
        and upper_carriage_pos is not None
        and upper_carriage_pos[1] > lower_carriage_pos[1] + 0.60
        and abs(upper_carriage_pos[0] - lower_carriage_pos[0]) < 1e-6
        and abs(upper_carriage_pos[2] - lower_carriage_pos[2]) < 1e-6,
        details=f"lower={lower_carriage_pos}, upper={upper_carriage_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
