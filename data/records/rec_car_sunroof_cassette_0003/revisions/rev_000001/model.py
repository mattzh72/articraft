from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)

FRAME_LENGTH = 1.26
FRAME_WIDTH = 0.84
FRAME_SIDE_RAIL_WIDTH = 0.08
FRAME_END_MEMBER_LENGTH = 0.08
FRAME_HOUSING_HEIGHT = 0.03
GUIDE_RAIL_WIDTH = 0.028
GUIDE_RAIL_HEIGHT = 0.004
GUIDE_RAIL_CENTER_Y = 0.346
GUIDE_RAIL_CENTER_Z = FRAME_HOUSING_HEIGHT + GUIDE_RAIL_HEIGHT / 2.0

CENTER_CROSSMEMBER_LENGTH = FRAME_WIDTH - 2.0 * FRAME_SIDE_RAIL_WIDTH
CENTER_CROSSMEMBER_THICKNESS = 0.06
CENTER_CROSSMEMBER_HEIGHT = FRAME_HOUSING_HEIGHT

PANEL_LENGTH = 0.46
PANEL_WIDTH = 0.74
PANEL_CARRIER_THICKNESS = 0.004
PANEL_GLASS_THICKNESS = 0.006
PANEL_CARRIER_CENTER_Z = 0.040
PANEL_GLASS_CENTER_Z = 0.045
PANEL_FRONT_EDGE_ANCHOR = (0.08, 0.12, 0.008)
PANEL_ANCHOR_CENTER_Z = 0.036
PANEL_FRONT_EDGE_ANCHOR_CENTER_X = PANEL_LENGTH / 2.0 - PANEL_FRONT_EDGE_ANCHOR[0] / 2.0 - 0.01

PANEL_REST_X = 0.31
FRONT_PANEL_TRAVEL = (-0.04, 0.0)
REAR_PANEL_TRAVEL = (0.0, 0.04)

SHOE_SIZE = (0.05, 0.026, 0.004)
SHOE_MOUNT_Z = FRAME_HOUSING_HEIGHT + GUIDE_RAIL_HEIGHT
SHOE_FRONT_REAR_OFFSET_X = 0.17
SHOE_LEFT_RIGHT_OFFSET_Y = GUIDE_RAIL_CENTER_Y


def _panel_inertial() -> Inertial:
    return Inertial.from_geometry(
        Box((PANEL_LENGTH, PANEL_WIDTH, 0.012)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
    )


def _add_guide_shoe(
    model: ArticulatedObject,
    parent_panel,
    shoe_name: str,
    mount_xyz: tuple[float, float, float],
    shoe_material,
):
    shoe = model.part(shoe_name)
    shoe.visual(
        Box(SHOE_SIZE),
        origin=Origin(xyz=(0.0, 0.0, SHOE_SIZE[2] / 2.0)),
        material=shoe_material,
        name="body",
    )
    shoe.inertial = Inertial.from_geometry(
        Box(SHOE_SIZE),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, SHOE_SIZE[2] / 2.0)),
    )
    model.articulation(
        f"{shoe_name}_mount",
        ArticulationType.FIXED,
        parent=parent_panel,
        child=shoe,
        origin=Origin(xyz=mount_xyz),
    )
    return shoe


def _add_panel(
    model: ArticulatedObject,
    frame,
    name: str,
    rest_x: float,
    motion_limits: tuple[float, float],
    carrier_material,
    glass_material,
    anchor_material,
):
    panel = model.part(name)
    panel.visual(
        Box((PANEL_LENGTH, PANEL_WIDTH, PANEL_CARRIER_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, PANEL_CARRIER_CENTER_Z)),
        material=carrier_material,
        name="carrier",
    )
    panel.visual(
        Box((PANEL_LENGTH - 0.02, PANEL_WIDTH - 0.04, PANEL_GLASS_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, PANEL_GLASS_CENTER_Z)),
        material=glass_material,
        name="glass",
    )
    panel.visual(
        Box(PANEL_FRONT_EDGE_ANCHOR),
        origin=Origin(
            xyz=(
                PANEL_FRONT_EDGE_ANCHOR_CENTER_X,
                0.0,
                PANEL_ANCHOR_CENTER_Z,
            )
        ),
        material=anchor_material,
        name="cable_anchor",
    )
    panel.inertial = _panel_inertial()
    model.articulation(
        f"{name}_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=panel,
        origin=Origin(xyz=(rest_x, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.25,
            lower=motion_limits[0],
            upper=motion_limits[1],
        ),
    )
    return panel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="panoramic_sunroof_cassette", assets=ASSETS)

    frame_material = model.material("frame_metal", rgba=(0.18, 0.19, 0.21, 1.0))
    crossmember_material = model.material("crossmember", rgba=(0.15, 0.16, 0.18, 1.0))
    carrier_material = model.material("panel_carrier", rgba=(0.08, 0.09, 0.10, 1.0))
    glass_material = model.material("glass", rgba=(0.36, 0.52, 0.60, 0.42))
    anchor_material = model.material("anchor", rgba=(0.44, 0.45, 0.47, 1.0))
    shoe_material = model.material("shoe", rgba=(0.12, 0.12, 0.13, 1.0))

    frame = model.part("cassette_frame")
    frame.visual(
        Box((FRAME_LENGTH, FRAME_SIDE_RAIL_WIDTH, FRAME_HOUSING_HEIGHT)),
        origin=Origin(xyz=(0.0, FRAME_WIDTH / 2.0 - FRAME_SIDE_RAIL_WIDTH / 2.0, FRAME_HOUSING_HEIGHT / 2.0)),
        material=frame_material,
        name="left_side_rail",
    )
    frame.visual(
        Box((FRAME_LENGTH, FRAME_SIDE_RAIL_WIDTH, FRAME_HOUSING_HEIGHT)),
        origin=Origin(xyz=(0.0, -FRAME_WIDTH / 2.0 + FRAME_SIDE_RAIL_WIDTH / 2.0, FRAME_HOUSING_HEIGHT / 2.0)),
        material=frame_material,
        name="right_side_rail",
    )
    frame.visual(
        Box((FRAME_END_MEMBER_LENGTH, FRAME_WIDTH, FRAME_HOUSING_HEIGHT)),
        origin=Origin(xyz=(FRAME_LENGTH / 2.0 - FRAME_END_MEMBER_LENGTH / 2.0, 0.0, FRAME_HOUSING_HEIGHT / 2.0)),
        material=frame_material,
        name="front_header",
    )
    frame.visual(
        Box((FRAME_END_MEMBER_LENGTH, FRAME_WIDTH, FRAME_HOUSING_HEIGHT)),
        origin=Origin(xyz=(-FRAME_LENGTH / 2.0 + FRAME_END_MEMBER_LENGTH / 2.0, 0.0, FRAME_HOUSING_HEIGHT / 2.0)),
        material=frame_material,
        name="rear_header",
    )
    frame.visual(
        Box((FRAME_LENGTH, GUIDE_RAIL_WIDTH, GUIDE_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, GUIDE_RAIL_CENTER_Y, GUIDE_RAIL_CENTER_Z)),
        material=frame_material,
        name="left_guide_rail",
    )
    frame.visual(
        Box((FRAME_LENGTH, GUIDE_RAIL_WIDTH, GUIDE_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, -GUIDE_RAIL_CENTER_Y, GUIDE_RAIL_CENTER_Z)),
        material=frame_material,
        name="right_guide_rail",
    )
    frame.inertial = Inertial.from_geometry(
        Box((FRAME_LENGTH, FRAME_WIDTH, FRAME_HOUSING_HEIGHT)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, FRAME_HOUSING_HEIGHT / 2.0)),
    )

    crossmember = model.part("center_crossmember")
    crossmember.visual(
        Box((CENTER_CROSSMEMBER_THICKNESS, CENTER_CROSSMEMBER_LENGTH, CENTER_CROSSMEMBER_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, CENTER_CROSSMEMBER_HEIGHT / 2.0)),
        material=crossmember_material,
        name="beam",
    )
    crossmember.inertial = Inertial.from_geometry(
        Box((CENTER_CROSSMEMBER_THICKNESS, CENTER_CROSSMEMBER_LENGTH, CENTER_CROSSMEMBER_HEIGHT)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, CENTER_CROSSMEMBER_HEIGHT / 2.0)),
    )
    model.articulation(
        "center_crossmember_mount",
        ArticulationType.FIXED,
        parent=frame,
        child=crossmember,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    front_panel = _add_panel(
        model,
        frame,
        "front_panel",
        PANEL_REST_X,
        FRONT_PANEL_TRAVEL,
        carrier_material,
        glass_material,
        anchor_material,
    )
    rear_panel = _add_panel(
        model,
        frame,
        "rear_panel",
        -PANEL_REST_X,
        REAR_PANEL_TRAVEL,
        carrier_material,
        glass_material,
        anchor_material,
    )

    shoe_mounts = (
        ("front_left", SHOE_FRONT_REAR_OFFSET_X, SHOE_LEFT_RIGHT_OFFSET_Y),
        ("front_right", SHOE_FRONT_REAR_OFFSET_X, -SHOE_LEFT_RIGHT_OFFSET_Y),
        ("rear_left", -SHOE_FRONT_REAR_OFFSET_X, SHOE_LEFT_RIGHT_OFFSET_Y),
        ("rear_right", -SHOE_FRONT_REAR_OFFSET_X, -SHOE_LEFT_RIGHT_OFFSET_Y),
    )
    for corner_name, shoe_x, shoe_y in shoe_mounts:
        _add_guide_shoe(
            model,
            front_panel,
            f"front_panel_{corner_name}_shoe",
            (shoe_x, shoe_y, SHOE_MOUNT_Z),
            shoe_material,
        )
        _add_guide_shoe(
            model,
            rear_panel,
            f"rear_panel_{corner_name}_shoe",
            (shoe_x, shoe_y, SHOE_MOUNT_Z),
            shoe_material,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    frame = object_model.get_part("cassette_frame")
    crossmember = object_model.get_part("center_crossmember")
    front_panel = object_model.get_part("front_panel")
    rear_panel = object_model.get_part("rear_panel")
    front_slide = object_model.get_articulation("front_panel_slide")
    rear_slide = object_model.get_articulation("rear_panel_slide")

    frame_front_header = frame.get_visual("front_header")
    left_guide_rail = frame.get_visual("left_guide_rail")
    right_guide_rail = frame.get_visual("right_guide_rail")
    beam = crossmember.get_visual("beam")
    front_carrier = front_panel.get_visual("carrier")
    rear_carrier = rear_panel.get_visual("carrier")
    front_anchor = front_panel.get_visual("cable_anchor")
    rear_anchor = rear_panel.get_visual("cable_anchor")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
    )
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_contact(crossmember, frame, elem_a=beam)
    ctx.expect_within(crossmember, frame, axes="xy", inner_elem=beam)

    ctx.expect_within(front_panel, frame, axes="xy", inner_elem=front_carrier)
    ctx.expect_within(rear_panel, frame, axes="xy", inner_elem=rear_carrier)
    ctx.expect_gap(
        front_panel,
        frame,
        axis="z",
        min_gap=0.003,
        max_gap=0.02,
        positive_elem=front_carrier,
    )
    ctx.expect_gap(
        rear_panel,
        frame,
        axis="z",
        min_gap=0.003,
        max_gap=0.02,
        positive_elem=rear_carrier,
    )

    ctx.expect_gap(
        front_panel,
        crossmember,
        axis="x",
        min_gap=0.02,
        positive_elem=front_carrier,
        negative_elem=beam,
    )
    ctx.expect_gap(
        crossmember,
        rear_panel,
        axis="x",
        min_gap=0.02,
        positive_elem=beam,
        negative_elem=rear_carrier,
    )

    ctx.expect_gap(
        frame,
        front_panel,
        axis="x",
        min_gap=0.015,
        max_gap=0.06,
        positive_elem=frame_front_header,
        negative_elem=front_anchor,
    )
    ctx.expect_gap(
        crossmember,
        rear_panel,
        axis="x",
        min_gap=0.02,
        max_gap=0.06,
        positive_elem=beam,
        negative_elem=rear_anchor,
    )

    shoe_specs = (
        ("front_panel_front_left_shoe", front_panel, front_carrier, left_guide_rail),
        ("front_panel_front_right_shoe", front_panel, front_carrier, right_guide_rail),
        ("front_panel_rear_left_shoe", front_panel, front_carrier, left_guide_rail),
        ("front_panel_rear_right_shoe", front_panel, front_carrier, right_guide_rail),
        ("rear_panel_front_left_shoe", rear_panel, rear_carrier, left_guide_rail),
        ("rear_panel_front_right_shoe", rear_panel, rear_carrier, right_guide_rail),
        ("rear_panel_rear_left_shoe", rear_panel, rear_carrier, left_guide_rail),
        ("rear_panel_rear_right_shoe", rear_panel, rear_carrier, right_guide_rail),
    )
    for shoe_name, panel, carrier, guide_rail in shoe_specs:
        shoe = object_model.get_part(shoe_name)
        shoe_body = shoe.get_visual("body")
        ctx.expect_contact(shoe, panel, elem_a=shoe_body, elem_b=carrier)
        ctx.expect_contact(shoe, frame, elem_a=shoe_body, elem_b=guide_rail)

    with ctx.pose({front_slide: -0.04}):
        ctx.expect_gap(
            front_panel,
            crossmember,
            axis="x",
            min_gap=0.01,
            positive_elem=front_carrier,
            negative_elem=beam,
        )
        ctx.expect_contact(
            object_model.get_part("front_panel_front_left_shoe"),
            frame,
            elem_a=object_model.get_part("front_panel_front_left_shoe").get_visual("body"),
            elem_b=left_guide_rail,
        )
        ctx.expect_gap(
            crossmember,
            rear_panel,
            axis="x",
            min_gap=0.02,
            positive_elem=beam,
            negative_elem=rear_carrier,
        )

    with ctx.pose({rear_slide: 0.04}):
        ctx.expect_gap(
            crossmember,
            rear_panel,
            axis="x",
            min_gap=0.01,
            positive_elem=beam,
            negative_elem=rear_carrier,
        )
        ctx.expect_contact(
            object_model.get_part("rear_panel_front_right_shoe"),
            frame,
            elem_a=object_model.get_part("rear_panel_front_right_shoe").get_visual("body"),
            elem_b=right_guide_rail,
        )
        ctx.expect_gap(
            front_panel,
            crossmember,
            axis="x",
            min_gap=0.02,
            positive_elem=front_carrier,
            negative_elem=beam,
        )

    with ctx.pose({front_slide: -0.03, rear_slide: 0.03}):
        ctx.expect_gap(
            front_panel,
            crossmember,
            axis="x",
            min_gap=0.015,
            positive_elem=front_carrier,
            negative_elem=beam,
        )
        ctx.expect_gap(
            crossmember,
            rear_panel,
            axis="x",
            min_gap=0.015,
            positive_elem=beam,
            negative_elem=rear_carrier,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
