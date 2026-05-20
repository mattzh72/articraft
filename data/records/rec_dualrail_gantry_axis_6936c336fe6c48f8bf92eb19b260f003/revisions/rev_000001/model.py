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


BASE_LENGTH = 1.20
RAIL_SPACING = 0.72
SUPPORT_WIDTH = 0.10
SUPPORT_HEIGHT = 0.07
RAIL_WIDTH = 0.05
RAIL_HEIGHT = 0.015
FOOT_WIDTH = 0.14
FOOT_DEPTH = 0.14
FOOT_HEIGHT = 0.012
CROSSMEMBER_WIDTH = 0.86
CROSSMEMBER_DEPTH = 0.10
CROSSMEMBER_HEIGHT = 0.05
CROSSMEMBER_Y = 0.50

BRIDGE_SHOE_WIDTH = 0.11
BRIDGE_SHOE_DEPTH = 0.18
BRIDGE_SHOE_HEIGHT = 0.045
UPRIGHT_WIDTH = 0.10
UPRIGHT_DEPTH = 0.16
UPRIGHT_HEIGHT = 0.20
CROSSBEAM_LENGTH = RAIL_SPACING - UPRIGHT_WIDTH
CROSSBEAM_DEPTH = 0.12
CROSSBEAM_HEIGHT = 0.08
CROSSBEAM_BOTTOM_Z = 0.19
GUIDE_RAIL_WIDTH = 0.055
GUIDE_RAIL_HEIGHT = 0.018

TRUCK_BLOCK_LENGTH = 0.18
TRUCK_BLOCK_DEPTH = 0.14
TRUCK_BLOCK_HEIGHT = 0.08
TRUCK_PLATE_WIDTH = 0.12
TRUCK_PLATE_DEPTH = 0.025
TRUCK_PLATE_HEIGHT = 0.18
TRUCK_NOSE_WIDTH = 0.08
TRUCK_NOSE_DEPTH = 0.05
TRUCK_NOSE_HEIGHT = 0.06

BRIDGE_TRAVEL = 0.44
TRUCK_TRAVEL = 0.22

RAIL_TOP_Z = FOOT_HEIGHT + SUPPORT_HEIGHT + RAIL_HEIGHT
GUIDE_RAIL_TOP_Z = CROSSBEAM_BOTTOM_Z + CROSSBEAM_HEIGHT + GUIDE_RAIL_HEIGHT


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _fuse_all(*solids: cq.Workplane) -> cq.Workplane:
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid, clean=False)
    return result


def _add_mesh_visual(
    part,
    shape: cq.Workplane,
    mesh_name: str,
    *,
    visual_name: str,
    material: str,
) -> None:
    part.visual(
        mesh_from_cadquery(shape, mesh_name, tolerance=0.0007, angular_tolerance=0.08),
        material=material,
        name=visual_name,
    )


def _build_base_frame() -> cq.Workplane:
    left_support = _box(
        (SUPPORT_WIDTH, BASE_LENGTH, SUPPORT_HEIGHT),
        (-RAIL_SPACING / 2.0, 0.0, FOOT_HEIGHT + SUPPORT_HEIGHT / 2.0),
    )
    right_support = _box(
        (SUPPORT_WIDTH, BASE_LENGTH, SUPPORT_HEIGHT),
        (RAIL_SPACING / 2.0, 0.0, FOOT_HEIGHT + SUPPORT_HEIGHT / 2.0),
    )
    front_crossmember = _box(
        (CROSSMEMBER_WIDTH, CROSSMEMBER_DEPTH, CROSSMEMBER_HEIGHT),
        (0.0, CROSSMEMBER_Y, FOOT_HEIGHT + CROSSMEMBER_HEIGHT / 2.0),
    )
    rear_crossmember = _box(
        (CROSSMEMBER_WIDTH, CROSSMEMBER_DEPTH, CROSSMEMBER_HEIGHT),
        (0.0, -CROSSMEMBER_Y, FOOT_HEIGHT + CROSSMEMBER_HEIGHT / 2.0),
    )

    foot_y = BASE_LENGTH / 2.0 - FOOT_DEPTH / 2.0 - 0.03
    feet = [
        _box((FOOT_WIDTH, FOOT_DEPTH, FOOT_HEIGHT), (-RAIL_SPACING / 2.0, foot_y, FOOT_HEIGHT / 2.0)),
        _box((FOOT_WIDTH, FOOT_DEPTH, FOOT_HEIGHT), (-RAIL_SPACING / 2.0, -foot_y, FOOT_HEIGHT / 2.0)),
        _box((FOOT_WIDTH, FOOT_DEPTH, FOOT_HEIGHT), (RAIL_SPACING / 2.0, foot_y, FOOT_HEIGHT / 2.0)),
        _box((FOOT_WIDTH, FOOT_DEPTH, FOOT_HEIGHT), (RAIL_SPACING / 2.0, -foot_y, FOOT_HEIGHT / 2.0)),
    ]

    return _fuse_all(left_support, right_support, front_crossmember, rear_crossmember, *feet)


def _build_base_rail(x_pos: float) -> cq.Workplane:
    return _box(
        (RAIL_WIDTH, BASE_LENGTH - 0.08, RAIL_HEIGHT),
        (x_pos, 0.0, FOOT_HEIGHT + SUPPORT_HEIGHT + RAIL_HEIGHT / 2.0),
    )


def _build_bridge_frame() -> cq.Workplane:
    left_shoe = _box(
        (BRIDGE_SHOE_WIDTH, BRIDGE_SHOE_DEPTH, BRIDGE_SHOE_HEIGHT),
        (-RAIL_SPACING / 2.0, 0.0, BRIDGE_SHOE_HEIGHT / 2.0),
    )
    right_shoe = _box(
        (BRIDGE_SHOE_WIDTH, BRIDGE_SHOE_DEPTH, BRIDGE_SHOE_HEIGHT),
        (RAIL_SPACING / 2.0, 0.0, BRIDGE_SHOE_HEIGHT / 2.0),
    )
    left_upright = _box(
        (UPRIGHT_WIDTH, UPRIGHT_DEPTH, UPRIGHT_HEIGHT),
        (-RAIL_SPACING / 2.0, 0.0, BRIDGE_SHOE_HEIGHT + UPRIGHT_HEIGHT / 2.0),
    )
    right_upright = _box(
        (UPRIGHT_WIDTH, UPRIGHT_DEPTH, UPRIGHT_HEIGHT),
        (RAIL_SPACING / 2.0, 0.0, BRIDGE_SHOE_HEIGHT + UPRIGHT_HEIGHT / 2.0),
    )
    crossbeam = _box(
        (CROSSBEAM_LENGTH, CROSSBEAM_DEPTH, CROSSBEAM_HEIGHT),
        (0.0, 0.0, CROSSBEAM_BOTTOM_Z + CROSSBEAM_HEIGHT / 2.0),
    )

    return _fuse_all(left_shoe, right_shoe, left_upright, right_upright, crossbeam)


def _build_bridge_guide_rail() -> cq.Workplane:
    return _box(
        (CROSSBEAM_LENGTH, GUIDE_RAIL_WIDTH, GUIDE_RAIL_HEIGHT),
        (0.0, 0.0, CROSSBEAM_BOTTOM_Z + CROSSBEAM_HEIGHT + GUIDE_RAIL_HEIGHT / 2.0),
    )


def _build_truck_body() -> cq.Workplane:
    block = _box(
        (TRUCK_BLOCK_LENGTH, TRUCK_BLOCK_DEPTH, TRUCK_BLOCK_HEIGHT),
        (0.0, 0.0, TRUCK_BLOCK_HEIGHT / 2.0),
    )
    plate_center_y = TRUCK_BLOCK_DEPTH / 2.0 + TRUCK_PLATE_DEPTH / 2.0
    plate = _box(
        (TRUCK_PLATE_WIDTH, TRUCK_PLATE_DEPTH, TRUCK_PLATE_HEIGHT),
        (0.0, plate_center_y, -0.03),
    )
    nose_center_y = plate_center_y + TRUCK_PLATE_DEPTH / 2.0 + TRUCK_NOSE_DEPTH / 2.0
    nose = _box(
        (TRUCK_NOSE_WIDTH, TRUCK_NOSE_DEPTH, TRUCK_NOSE_HEIGHT),
        (0.0, nose_center_y, -0.11),
    )

    return _fuse_all(block, plate, nose)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_rail_gantry_axis")

    model.material("frame_dark", rgba=(0.19, 0.21, 0.23, 1.0))
    model.material("anodized_beam", rgba=(0.68, 0.70, 0.73, 1.0))
    model.material("rail_steel", rgba=(0.80, 0.82, 0.85, 1.0))
    model.material("carriage_gray", rgba=(0.56, 0.58, 0.61, 1.0))

    base = model.part("base")
    _add_mesh_visual(base, _build_base_frame(), "gantry_base_frame", visual_name="base_frame", material="frame_dark")
    _add_mesh_visual(
        base,
        _build_base_rail(-RAIL_SPACING / 2.0),
        "gantry_left_rail",
        visual_name="left_rail",
        material="rail_steel",
    )
    _add_mesh_visual(
        base,
        _build_base_rail(RAIL_SPACING / 2.0),
        "gantry_right_rail",
        visual_name="right_rail",
        material="rail_steel",
    )
    base.inertial = Inertial.from_geometry(
        Box((CROSSMEMBER_WIDTH, BASE_LENGTH, RAIL_TOP_Z)),
        mass=26.0,
        origin=Origin(xyz=(0.0, 0.0, RAIL_TOP_Z / 2.0)),
    )

    bridge = model.part("bridge")
    _add_mesh_visual(bridge, _build_bridge_frame(), "gantry_bridge_frame", visual_name="bridge_frame", material="anodized_beam")
    _add_mesh_visual(
        bridge,
        _build_bridge_guide_rail(),
        "gantry_bridge_guide",
        visual_name="guide_rail",
        material="rail_steel",
    )
    bridge.inertial = Inertial.from_geometry(
        Box((RAIL_SPACING + BRIDGE_SHOE_WIDTH, BRIDGE_SHOE_DEPTH, GUIDE_RAIL_TOP_Z)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, GUIDE_RAIL_TOP_Z / 2.0)),
    )

    truck = model.part("truck")
    _add_mesh_visual(truck, _build_truck_body(), "gantry_truck_body", visual_name="truck_body", material="carriage_gray")
    truck.inertial = Inertial.from_geometry(
        Box((TRUCK_BLOCK_LENGTH, 0.17, 0.22)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.035, -0.01)),
    )

    model.articulation(
        "base_to_bridge",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bridge,
        origin=Origin(xyz=(0.0, 0.0, RAIL_TOP_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-BRIDGE_TRAVEL,
            upper=BRIDGE_TRAVEL,
            effort=1800.0,
            velocity=0.8,
        ),
    )
    model.articulation(
        "bridge_to_truck",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=truck,
        origin=Origin(xyz=(0.0, 0.0, GUIDE_RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-TRUCK_TRAVEL,
            upper=TRUCK_TRAVEL,
            effort=500.0,
            velocity=0.6,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    bridge = object_model.get_part("bridge")
    truck = object_model.get_part("truck")
    bridge_slide = object_model.get_articulation("base_to_bridge")
    truck_slide = object_model.get_articulation("bridge_to_truck")

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
        "bridge axis is prismatic along base rails",
        bridge_slide.articulation_type == ArticulationType.PRISMATIC and bridge_slide.axis == (0.0, 1.0, 0.0),
        details=f"type={bridge_slide.articulation_type}, axis={bridge_slide.axis}",
    )
    ctx.check(
        "truck axis is prismatic across crossbeam",
        truck_slide.articulation_type == ArticulationType.PRISMATIC and truck_slide.axis == (1.0, 0.0, 0.0),
        details=f"type={truck_slide.articulation_type}, axis={truck_slide.axis}",
    )

    ctx.expect_contact(
        bridge,
        base,
        elem_b="left_rail",
        contact_tol=0.001,
        name="bridge contacts left base rail",
    )
    ctx.expect_contact(
        bridge,
        base,
        elem_b="right_rail",
        contact_tol=0.001,
        name="bridge contacts right base rail",
    )
    ctx.expect_contact(
        truck,
        bridge,
        elem_b="guide_rail",
        contact_tol=0.001,
        name="truck rides on bridge guide rail",
    )

    with ctx.pose({bridge_slide: bridge_slide.motion_limits.upper}):
        ctx.expect_origin_gap(
            bridge,
            base,
            axis="y",
            min_gap=BRIDGE_TRAVEL - 0.01,
            max_gap=BRIDGE_TRAVEL + 0.01,
            name="bridge moves in +y at upper limit",
        )
        ctx.expect_contact(
            bridge,
            base,
            elem_b="left_rail",
            contact_tol=0.001,
            name="bridge stays supported on left rail at upper limit",
        )

    with ctx.pose({bridge_slide: bridge_slide.motion_limits.lower}):
        ctx.expect_origin_gap(
            base,
            bridge,
            axis="y",
            min_gap=BRIDGE_TRAVEL - 0.01,
            max_gap=BRIDGE_TRAVEL + 0.01,
            name="bridge moves in -y at lower limit",
        )

    with ctx.pose({truck_slide: truck_slide.motion_limits.upper}):
        ctx.expect_origin_gap(
            truck,
            bridge,
            axis="x",
            min_gap=TRUCK_TRAVEL - 0.01,
            max_gap=TRUCK_TRAVEL + 0.01,
            name="truck moves in +x at upper limit",
        )
        ctx.expect_contact(
            truck,
            bridge,
            elem_b="guide_rail",
            contact_tol=0.001,
            name="truck stays on bridge guide rail at upper limit",
        )

    with ctx.pose({truck_slide: truck_slide.motion_limits.lower}):
        ctx.expect_origin_gap(
            bridge,
            truck,
            axis="x",
            min_gap=TRUCK_TRAVEL - 0.01,
            max_gap=TRUCK_TRAVEL + 0.01,
            name="truck moves in -x at lower limit",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
