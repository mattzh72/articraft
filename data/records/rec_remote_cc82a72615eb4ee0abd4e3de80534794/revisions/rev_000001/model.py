from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="universal_remote")

    shell_mat = model.material("soft_black_plastic", rgba=(0.035, 0.038, 0.042, 1.0))
    panel_mat = model.material("matte_button_panel", rgba=(0.015, 0.016, 0.018, 1.0))
    rubber_mat = model.material("dark_rubber_buttons", rgba=(0.025, 0.025, 0.027, 1.0))
    grey_button_mat = model.material("grey_rubber_buttons", rgba=(0.23, 0.24, 0.25, 1.0))
    red_button_mat = model.material("red_power_button", rgba=(0.78, 0.05, 0.04, 1.0))
    label_mat = model.material("pale_printed_labels", rgba=(0.82, 0.84, 0.80, 1.0))
    chrome_mat = model.material("brushed_chrome", rgba=(0.72, 0.74, 0.76, 1.0))
    hinge_mat = model.material("black_hinge_pin", rgba=(0.010, 0.011, 0.012, 1.0))
    door_mat = model.material("rear_battery_door", rgba=(0.055, 0.058, 0.064, 1.0))

    body = model.part("body")
    body_profile = rounded_rect_profile(0.065, 0.190, 0.013, corner_segments=10)
    body.visual(
        mesh_from_geometry(ExtrudeGeometry.centered(body_profile, 0.020), "rounded_remote_body"),
        material=shell_mat,
        name="shell",
    )

    # Raised face panel and a clear universal-remote button layout.
    body.visual(
        Box((0.052, 0.135, 0.0016)),
        origin=Origin(xyz=(0.000, -0.018, 0.01065)),
        material=panel_mat,
        name="button_face",
    )
    body.visual(
        Box((0.034, 0.014, 0.0012)),
        origin=Origin(xyz=(0.000, 0.055, 0.01180)),
        material=label_mat,
        name="label_window",
    )
    body.visual(
        Cylinder(radius=0.0060, length=0.0024),
        origin=Origin(xyz=(-0.018, 0.071, 0.01110)),
        material=red_button_mat,
        name="power_button",
    )
    body.visual(
        Cylinder(radius=0.0042, length=0.0022),
        origin=Origin(xyz=(0.018, 0.071, 0.01100)),
        material=grey_button_mat,
        name="input_button",
    )
    body.visual(
        Cylinder(radius=0.0115, length=0.0020),
        origin=Origin(xyz=(0.000, 0.023, 0.01205)),
        material=grey_button_mat,
        name="navigation_ring",
    )
    body.visual(
        Cylinder(radius=0.0052, length=0.0026),
        origin=Origin(xyz=(0.000, 0.023, 0.01255)),
        material=rubber_mat,
        name="select_button",
    )
    for index, (x, y) in enumerate(
        (
            (0.000, 0.038),
            (0.000, 0.008),
            (-0.015, 0.023),
            (0.015, 0.023),
        )
    ):
        body.visual(
            Box((0.011 if x else 0.016, 0.007 if x else 0.008, 0.0020)),
            origin=Origin(xyz=(x, y, 0.01225)),
            material=rubber_mat,
            name=f"nav_key_{index}",
        )
    for row, y in enumerate((-0.004, -0.023, -0.042)):
        for col, x in enumerate((-0.018, 0.000, 0.018)):
            body.visual(
                Cylinder(radius=0.0050, length=0.0022),
                origin=Origin(xyz=(x, y, 0.01210)),
                material=grey_button_mat,
                name=f"number_button_{row}_{col}",
            )
    for col, x in enumerate((-0.018, 0.000, 0.018)):
        body.visual(
            Box((0.011, 0.007, 0.0020)),
            origin=Origin(xyz=(x, -0.061, 0.01210)),
            material=rubber_mat,
            name=f"transport_button_{col}",
        )

    # Telescoping antenna socket mounted along the top edge, just proud of the face.
    body.visual(
        Box((0.020, 0.034, 0.0040)),
        origin=Origin(xyz=(0.000, 0.103, 0.0120)),
        material=shell_mat,
        name="antenna_saddle",
    )
    body.visual(
        Cylinder(radius=0.0055, length=0.040),
        origin=Origin(xyz=(0.000, 0.108, 0.0180), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=hinge_mat,
        name="antenna_socket",
    )

    # Rear battery door frame and fixed hinge knuckles on the back face.
    body.visual(
        Box((0.004, 0.095, 0.0008)),
        origin=Origin(xyz=(-0.026, -0.020, -0.0104)),
        material=panel_mat,
        name="battery_frame_hinge_side",
    )
    body.visual(
        Box((0.004, 0.095, 0.0008)),
        origin=Origin(xyz=(0.022, -0.020, -0.0104)),
        material=panel_mat,
        name="battery_frame_latch_side",
    )
    body.visual(
        Box((0.048, 0.004, 0.0008)),
        origin=Origin(xyz=(-0.002, 0.028, -0.0104)),
        material=panel_mat,
        name="battery_frame_top",
    )
    body.visual(
        Box((0.048, 0.004, 0.0008)),
        origin=Origin(xyz=(-0.002, -0.068, -0.0104)),
        material=panel_mat,
        name="battery_frame_bottom",
    )
    for index, y in enumerate((-0.054, 0.014)):
        body.visual(
            Box((0.007, 0.016, 0.0022)),
            origin=Origin(xyz=(-0.022, y, -0.0110)),
            material=hinge_mat,
            name=f"rear_hinge_leaf_{index}",
        )
        body.visual(
            Cylinder(radius=0.0022, length=0.016),
            origin=Origin(xyz=(-0.022, y, -0.0130), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=hinge_mat,
            name=f"rear_hinge_knuckle_{index}",
        )

    antenna = model.part("antenna")
    antenna.visual(
        Cylinder(radius=0.0032, length=0.200),
        origin=Origin(xyz=(0.0, 0.000, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=chrome_mat,
        name="base_stage",
    )
    antenna.visual(
        Cylinder(radius=0.0020, length=0.130),
        origin=Origin(xyz=(0.0, 0.155, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=chrome_mat,
        name="tip_stage",
    )
    antenna.visual(
        Sphere(radius=0.0032),
        origin=Origin(xyz=(0.0, 0.222, 0.0)),
        material=chrome_mat,
        name="rounded_tip",
    )

    battery_door = model.part("battery_door")
    battery_door.visual(
        Box((0.040, 0.085, 0.0020)),
        origin=Origin(xyz=(0.020, 0.000, -0.0010)),
        material=door_mat,
        name="door_panel",
    )
    battery_door.visual(
        Cylinder(radius=0.0022, length=0.040),
        origin=Origin(xyz=(0.000, 0.000, -0.0030), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=hinge_mat,
        name="door_hinge_barrel",
    )
    battery_door.visual(
        Box((0.008, 0.030, 0.0009)),
        origin=Origin(xyz=(0.034, 0.000, -0.00245)),
        material=panel_mat,
        name="finger_grip",
    )
    battery_door.visual(
        Box((0.0025, 0.020, 0.0014)),
        origin=Origin(xyz=(0.0405, 0.000, -0.0013)),
        material=door_mat,
        name="latch_lip",
    )

    model.articulation(
        "antenna_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=antenna,
        origin=Origin(xyz=(0.000, 0.128, 0.0180)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.35, lower=0.0, upper=0.080),
    )
    model.articulation(
        "battery_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=battery_door,
        origin=Origin(xyz=(-0.022, -0.020, -0.0100)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=2.0, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    antenna = object_model.get_part("antenna")
    battery_door = object_model.get_part("battery_door")
    antenna_slide = object_model.get_articulation("antenna_slide")
    door_hinge = object_model.get_articulation("battery_door_hinge")

    ctx.check(
        "exactly two requested joints",
        len(object_model.articulations) == 2,
        details=f"found {[joint.name for joint in object_model.articulations]}",
    )

    ctx.allow_overlap(
        body,
        antenna,
        elem_a="antenna_socket",
        elem_b="base_stage",
        reason="The moving antenna base stage is intentionally captured inside the fixed top-edge socket.",
    )
    ctx.expect_within(
        antenna,
        body,
        axes="xz",
        inner_elem="base_stage",
        outer_elem="antenna_socket",
        margin=0.0005,
        name="antenna base centered in socket",
    )
    ctx.expect_overlap(
        antenna,
        body,
        axes="y",
        elem_a="base_stage",
        elem_b="antenna_socket",
        min_overlap=0.035,
        name="antenna retained in socket at rest",
    )

    ctx.expect_gap(
        body,
        battery_door,
        axis="z",
        max_gap=0.001,
        max_penetration=0.000001,
        positive_elem="shell",
        negative_elem="door_panel",
        name="battery door sits flush on rear face",
    )
    ctx.expect_within(
        battery_door,
        body,
        axes="xy",
        inner_elem="door_panel",
        outer_elem="shell",
        margin=0.002,
        name="battery door fits within rear body footprint",
    )

    rest_antenna_pos = ctx.part_world_position(antenna)
    with ctx.pose({antenna_slide: 0.080}):
        ctx.expect_within(
            antenna,
            body,
            axes="xz",
            inner_elem="base_stage",
            outer_elem="antenna_socket",
            margin=0.0005,
            name="extended antenna remains centered in socket",
        )
        ctx.expect_overlap(
            antenna,
            body,
            axes="y",
            elem_a="base_stage",
            elem_b="antenna_socket",
            min_overlap=0.015,
            name="extended antenna keeps retained insertion",
        )
        extended_antenna_pos = ctx.part_world_position(antenna)
    ctx.check(
        "antenna extends from top edge",
        rest_antenna_pos is not None
        and extended_antenna_pos is not None
        and extended_antenna_pos[1] > rest_antenna_pos[1] + 0.070,
        details=f"rest={rest_antenna_pos}, extended={extended_antenna_pos}",
    )

    closed_door_aabb = ctx.part_world_aabb(battery_door)
    with ctx.pose({door_hinge: 1.20}):
        open_door_aabb = ctx.part_world_aabb(battery_door)
    ctx.check(
        "battery door swings outward from rear",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][2] < closed_door_aabb[0][2] - 0.015,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
