from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


RAIL_LENGTH = 1.60
RAIL_SPACING = 0.90
RAIL_BASE_WIDTH = 0.12
RAIL_BASE_HEIGHT = 0.045
RAIL_SHOULDER_WIDTH = 0.09
RAIL_SHOULDER_HEIGHT = 0.020
TRACK_WIDTH = 0.065
TRACK_THICKNESS = 0.012
TRACK_TOP_Z = RAIL_BASE_HEIGHT + RAIL_SHOULDER_HEIGHT + TRACK_THICKNESS

CROSS_TIE_WIDTH = 1.10
CROSS_TIE_DEPTH = 0.12
CROSS_TIE_HEIGHT = 0.050
CROSS_TIE_Y = 0.62

BRIDGE_HOME_Z = 0.46
BRIDGE_TRAVEL = 0.55
BRIDGE_BEAM_LENGTH = 1.06
BRIDGE_BEAM_DEPTH = 0.16
BRIDGE_BEAM_HEIGHT = 0.18
BRIDGE_GUIDE_LENGTH = 0.78
BRIDGE_GUIDE_WIDTH = 0.10
BRIDGE_GUIDE_HEIGHT = 0.036
BRIDGE_GUIDE_CENTER_Z = (
    (BRIDGE_BEAM_HEIGHT / 2.0) + (BRIDGE_GUIDE_HEIGHT / 2.0) - 0.001
)

SHOE_WIDTH_X = 0.18
SHOE_LENGTH_Y = 0.20
SHOE_HEIGHT = 0.035
SHOE_CENTER_Z_LOCAL = TRACK_TOP_Z + (SHOE_HEIGHT / 2.0) - BRIDGE_HOME_Z

RISER_WIDTH_X = 0.16
RISER_DEPTH_Y = 0.14
RISER_BOTTOM_WORLD_Z = TRACK_TOP_Z + SHOE_HEIGHT - 0.002
RISER_TOP_WORLD_Z = BRIDGE_HOME_Z - (BRIDGE_BEAM_HEIGHT / 2.0) + 0.002
RISER_HEIGHT = RISER_TOP_WORLD_Z - RISER_BOTTOM_WORLD_Z
RISER_CENTER_Z_LOCAL = ((RISER_BOTTOM_WORLD_Z + RISER_TOP_WORLD_Z) / 2.0) - BRIDGE_HOME_Z

TRUCK_TRAVEL = 0.25
TRUCK_BODY_LENGTH_X = 0.22
TRUCK_BODY_DEPTH_Y = 0.18
TRUCK_BODY_HEIGHT = 0.16
TRUCK_BODY_CENTER_Z = 0.10
TRUCK_SLOT_LENGTH_X = 0.40
TRUCK_SLOT_DEPTH_Y = 0.114
TRUCK_SLOT_HEIGHT = 0.085
TRUCK_SLOT_CENTER_Z = 0.015
BEARING_PAD_LENGTH_X = 0.18
BEARING_PAD_DEPTH_Y = 0.08
BEARING_PAD_HEIGHT = 0.040
BEARING_PAD_CENTER_Z = (BRIDGE_GUIDE_HEIGHT / 2.0) + (BEARING_PAD_HEIGHT / 2.0)
FACEPLATE_WIDTH_X = 0.18
FACEPLATE_THICKNESS = 0.014
FACEPLATE_HEIGHT = 0.20
FACEPLATE_CENTER_Y = (
    (TRUCK_BODY_DEPTH_Y / 2.0) + (FACEPLATE_THICKNESS / 2.0) - 0.001
)
FACEPLATE_CENTER_Z = 0.10


def _box(
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    *,
    fillet: float = 0.0,
):
    solid = cq.Workplane("XY").box(*size)
    if fillet > 0.0:
        solid = solid.edges("|Z").fillet(fillet)
    return solid.translate(center)


def _base_frame_shape():
    rail_x = RAIL_SPACING / 2.0
    frame = _box(
        (RAIL_BASE_WIDTH, RAIL_LENGTH, RAIL_BASE_HEIGHT),
        (-rail_x, 0.0, RAIL_BASE_HEIGHT / 2.0),
        fillet=0.005,
    )
    frame = frame.union(
        _box(
            (RAIL_BASE_WIDTH, RAIL_LENGTH, RAIL_BASE_HEIGHT),
            (rail_x, 0.0, RAIL_BASE_HEIGHT / 2.0),
            fillet=0.005,
        )
    )
    frame = frame.union(
        _box(
            (RAIL_SHOULDER_WIDTH, RAIL_LENGTH, RAIL_SHOULDER_HEIGHT),
            (
                -rail_x,
                0.0,
                RAIL_BASE_HEIGHT + (RAIL_SHOULDER_HEIGHT / 2.0) + 0.001,
            ),
            fillet=0.004,
        )
    )
    frame = frame.union(
        _box(
            (RAIL_SHOULDER_WIDTH, RAIL_LENGTH, RAIL_SHOULDER_HEIGHT),
            (
                rail_x,
                0.0,
                RAIL_BASE_HEIGHT + (RAIL_SHOULDER_HEIGHT / 2.0) + 0.001,
            ),
            fillet=0.004,
        )
    )
    frame = frame.union(
        _box(
            (CROSS_TIE_WIDTH, CROSS_TIE_DEPTH, CROSS_TIE_HEIGHT),
            (0.0, -CROSS_TIE_Y, CROSS_TIE_HEIGHT / 2.0),
            fillet=0.006,
        )
    )
    frame = frame.union(
        _box(
            (CROSS_TIE_WIDTH, CROSS_TIE_DEPTH, CROSS_TIE_HEIGHT),
            (0.0, CROSS_TIE_Y, CROSS_TIE_HEIGHT / 2.0),
            fillet=0.006,
        )
    )
    return frame


def _bridge_body_shape():
    rail_x = RAIL_SPACING / 2.0
    bridge = _box(
        (BRIDGE_BEAM_LENGTH, BRIDGE_BEAM_DEPTH, BRIDGE_BEAM_HEIGHT),
        (0.0, 0.0, 0.0),
        fillet=0.012,
    )
    bridge = bridge.union(
        _box(
            (RISER_WIDTH_X, RISER_DEPTH_Y, RISER_HEIGHT),
            (-rail_x, 0.0, RISER_CENTER_Z_LOCAL),
            fillet=0.008,
        )
    )
    bridge = bridge.union(
        _box(
            (RISER_WIDTH_X, RISER_DEPTH_Y, RISER_HEIGHT),
            (rail_x, 0.0, RISER_CENTER_Z_LOCAL),
            fillet=0.008,
        )
    )
    return bridge


def _truck_body_shape():
    truck = _box(
        (TRUCK_BODY_LENGTH_X, TRUCK_BODY_DEPTH_Y, TRUCK_BODY_HEIGHT),
        (0.0, 0.0, TRUCK_BODY_CENTER_Z),
        fillet=0.008,
    )
    slot = _box(
        (TRUCK_SLOT_LENGTH_X, TRUCK_SLOT_DEPTH_Y, TRUCK_SLOT_HEIGHT),
        (0.0, 0.0, TRUCK_SLOT_CENTER_Z),
    )
    return truck.cut(slot)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_on_rails_gantry_axis")

    model.material("base_gray", rgba=(0.26, 0.28, 0.30, 1.0))
    model.material("rail_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("bridge_gray", rgba=(0.77, 0.79, 0.82, 1.0))
    model.material("truck_dark", rgba=(0.22, 0.23, 0.26, 1.0))
    model.material("faceplate_light", rgba=(0.84, 0.85, 0.87, 1.0))

    rail_x = RAIL_SPACING / 2.0

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_frame_shape(), "gantry_base_frame"),
        material="base_gray",
        name="base_frame",
    )
    base.visual(
        Box((TRACK_WIDTH, RAIL_LENGTH, TRACK_THICKNESS)),
        origin=Origin(
            xyz=(
                -rail_x,
                0.0,
                RAIL_BASE_HEIGHT + RAIL_SHOULDER_HEIGHT + (TRACK_THICKNESS / 2.0),
            )
        ),
        material="rail_steel",
        name="left_track",
    )
    base.visual(
        Box((TRACK_WIDTH, RAIL_LENGTH, TRACK_THICKNESS)),
        origin=Origin(
            xyz=(
                rail_x,
                0.0,
                RAIL_BASE_HEIGHT + RAIL_SHOULDER_HEIGHT + (TRACK_THICKNESS / 2.0),
            )
        ),
        material="rail_steel",
        name="right_track",
    )

    bridge = model.part("bridge")
    bridge.visual(
        mesh_from_cadquery(_bridge_body_shape(), "gantry_bridge_body"),
        material="bridge_gray",
        name="bridge_body",
    )
    bridge.visual(
        Box((SHOE_WIDTH_X, SHOE_LENGTH_Y, SHOE_HEIGHT)),
        origin=Origin(xyz=(-rail_x, 0.0, SHOE_CENTER_Z_LOCAL)),
        material="bridge_gray",
        name="left_shoe",
    )
    bridge.visual(
        Box((SHOE_WIDTH_X, SHOE_LENGTH_Y, SHOE_HEIGHT)),
        origin=Origin(xyz=(rail_x, 0.0, SHOE_CENTER_Z_LOCAL)),
        material="bridge_gray",
        name="right_shoe",
    )
    bridge.visual(
        Box((BRIDGE_GUIDE_LENGTH, BRIDGE_GUIDE_WIDTH, BRIDGE_GUIDE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BRIDGE_GUIDE_CENTER_Z)),
        material="rail_steel",
        name="cross_guide",
    )

    truck = model.part("truck")
    truck.visual(
        mesh_from_cadquery(_truck_body_shape(), "gantry_truck_body"),
        material="truck_dark",
        name="truck_body",
    )
    truck.visual(
        Box((BEARING_PAD_LENGTH_X, BEARING_PAD_DEPTH_Y, BEARING_PAD_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BEARING_PAD_CENTER_Z)),
        material="rail_steel",
        name="bearing_pad",
    )
    truck.visual(
        Box((FACEPLATE_WIDTH_X, FACEPLATE_THICKNESS, FACEPLATE_HEIGHT)),
        origin=Origin(xyz=(0.0, FACEPLATE_CENTER_Y, FACEPLATE_CENTER_Z)),
        material="faceplate_light",
        name="faceplate",
    )

    model.articulation(
        "base_to_bridge",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bridge,
        origin=Origin(xyz=(0.0, 0.0, BRIDGE_HOME_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=800.0,
            velocity=0.4,
            lower=-BRIDGE_TRAVEL,
            upper=BRIDGE_TRAVEL,
        ),
    )
    model.articulation(
        "bridge_to_truck",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=truck,
        origin=Origin(xyz=(0.0, 0.0, BRIDGE_GUIDE_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=0.35,
            lower=-TRUCK_TRAVEL,
            upper=TRUCK_TRAVEL,
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
    truck = object_model.get_part("truck")
    bridge_slide = object_model.get_articulation("base_to_bridge")
    truck_slide = object_model.get_articulation("bridge_to_truck")

    left_track = base.get_visual("left_track")
    right_track = base.get_visual("right_track")
    bridge_body = bridge.get_visual("bridge_body")
    left_shoe = bridge.get_visual("left_shoe")
    right_shoe = bridge.get_visual("right_shoe")
    cross_guide = bridge.get_visual("cross_guide")
    truck_body = truck.get_visual("truck_body")
    bearing_pad = truck.get_visual("bearing_pad")
    faceplate = truck.get_visual("faceplate")

    ctx.check("base present", base is not None)
    ctx.check("bridge present", bridge is not None)
    ctx.check("truck present", truck is not None)

    with ctx.pose({bridge_slide: 0.0, truck_slide: 0.0}):
        ctx.expect_contact(
            bridge,
            base,
            elem_a=left_shoe,
            elem_b=left_track,
            contact_tol=0.0015,
            name="left bridge shoe sits on left rail",
        )
        ctx.expect_contact(
            bridge,
            base,
            elem_a=right_shoe,
            elem_b=right_track,
            contact_tol=0.0015,
            name="right bridge shoe sits on right rail",
        )
        ctx.expect_overlap(
            truck,
            bridge,
            axes="x",
            elem_a=truck_body,
            elem_b=cross_guide,
            min_overlap=0.20,
            name="truck overlaps the cross guide at home",
        )
        ctx.expect_contact(
            truck,
            bridge,
            elem_a=bearing_pad,
            elem_b=cross_guide,
            contact_tol=0.0015,
            name="truck bearing pad rests on the cross guide",
        )
        ctx.expect_gap(
            truck,
            bridge,
            axis="y",
            positive_elem=faceplate,
            negative_elem=bridge_body,
            min_gap=0.006,
            name="faceplate stands proud of the beam front",
        )

        bridge_home = ctx.part_world_position(bridge)
        truck_home = ctx.part_world_position(truck)

    with ctx.pose({bridge_slide: BRIDGE_TRAVEL}):
        ctx.expect_contact(
            bridge,
            base,
            elem_a=left_shoe,
            elem_b=left_track,
            contact_tol=0.0015,
            name="left bridge shoe remains on the left rail at full bridge travel",
        )
        ctx.expect_contact(
            bridge,
            base,
            elem_a=right_shoe,
            elem_b=right_track,
            contact_tol=0.0015,
            name="right bridge shoe remains on the right rail at full bridge travel",
        )
        bridge_forward = ctx.part_world_position(bridge)

    with ctx.pose({truck_slide: TRUCK_TRAVEL}):
        ctx.expect_overlap(
            truck,
            bridge,
            axes="x",
            elem_a=truck_body,
            elem_b=cross_guide,
            min_overlap=0.15,
            name="truck retains guide engagement at full cross travel",
        )
        truck_right = ctx.part_world_position(truck)

    ctx.check(
        "bridge travels along the rail axis",
        bridge_home is not None
        and bridge_forward is not None
        and bridge_forward[1] > bridge_home[1] + 0.40,
        details=f"home={bridge_home}, forward={bridge_forward}",
    )
    ctx.check(
        "truck travels across the bridge",
        truck_home is not None
        and truck_right is not None
        and truck_right[0] > truck_home[0] + 0.18
        and abs(truck_right[1] - truck_home[1]) < 0.01,
        details=f"home={truck_home}, right={truck_right}",
    )

    with ctx.pose({bridge_slide: BRIDGE_TRAVEL, truck_slide: TRUCK_TRAVEL}):
        ctx.expect_contact(
            bridge,
            base,
            elem_a=left_shoe,
            elem_b=left_track,
            contact_tol=0.0015,
            name="left shoe stays supported in the combined max pose",
        )
        ctx.expect_contact(
            bridge,
            base,
            elem_a=right_shoe,
            elem_b=right_track,
            contact_tol=0.0015,
            name="right shoe stays supported in the combined max pose",
        )
        ctx.expect_overlap(
            truck,
            bridge,
            axes="x",
            elem_a=truck_body,
            elem_b=cross_guide,
            min_overlap=0.15,
            name="truck stays engaged with the guide in the combined max pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
