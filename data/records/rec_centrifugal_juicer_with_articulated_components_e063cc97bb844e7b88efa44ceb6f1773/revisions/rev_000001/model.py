from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _lathe_tube(inner_radius: float, outer_radius: float, z_min: float, z_max: float, *, segments: int = 64):
    """A closed thin-walled tube made from a revolved rectangular section."""
    return LatheGeometry(
        [
            (inner_radius, z_min),
            (outer_radius, z_min),
            (outer_radius, z_max),
            (inner_radius, z_max),
        ],
        segments=segments,
        closed=True,
    )


def _merged_filter_basket() -> MeshGeometry:
    """Rotating stainless basket: conical screen, central hub, and four spokes."""
    basket = MeshGeometry()
    basket.merge(
        LatheGeometry(
            [
                (0.044, 0.004),
                (0.060, 0.004),
                (0.106, 0.087),
                (0.100, 0.098),
                (0.084, 0.092),
                (0.048, 0.014),
            ],
            segments=72,
            closed=True,
        )
    )
    basket.merge(CylinderGeometry(0.025, 0.026, radial_segments=40).translate(0.0, 0.0, 0.013))
    for i in range(4):
        spoke = BoxGeometry((0.080, 0.007, 0.008)).translate(0.043, 0.0, 0.018)
        basket.merge(spoke.rotate_z(i * pi / 2.0))
    return basket


def _merged_lid() -> MeshGeometry:
    """Transparent shallow lid with a central hollow chute."""
    lid = MeshGeometry()
    cover = LatheGeometry(
        [
            (0.045, 0.002),
            (0.150, 0.002),
            (0.158, 0.010),
            (0.143, 0.027),
            (0.092, 0.043),
            (0.047, 0.036),
        ],
        segments=72,
        closed=True,
    ).translate(0.0, -0.150, 0.0)
    chute = _lathe_tube(0.034, 0.047, 0.020, 0.176, segments=64).translate(0.0, -0.150, 0.0)
    lid.merge(cover)
    lid.merge(chute)
    return lid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_kitchen_juicer")

    model.material("warm_white", rgba=(0.92, 0.90, 0.84, 1.0))
    model.material("soft_gray", rgba=(0.55, 0.57, 0.58, 1.0))
    model.material("dark_gray", rgba=(0.08, 0.09, 0.10, 1.0))
    model.material("brushed_steel", rgba=(0.72, 0.75, 0.76, 1.0))
    model.material("smoked_clear", rgba=(0.62, 0.86, 0.98, 0.38))
    model.material("pusher_frosted", rgba=(0.80, 0.85, 0.88, 0.72))
    model.material("orange_pulp", rgba=(0.94, 0.52, 0.14, 1.0))

    body = model.part("body")
    body.visual(Box((0.380, 0.340, 0.130)), origin=Origin(xyz=(0.0, 0.0, 0.065)), material="warm_white", name="compact_base")
    body.visual(Box((0.360, 0.310, 0.030)), origin=Origin(xyz=(0.0, 0.0, 0.140)), material="soft_gray", name="top_band")
    body.visual(
        mesh_from_geometry(
            LatheGeometry(
                [
                    (0.104, 0.140),
                    (0.155, 0.140),
                    (0.170, 0.184),
                    (0.162, 0.264),
                    (0.145, 0.275),
                    (0.122, 0.254),
                    (0.126, 0.180),
                    (0.104, 0.150),
                ],
                segments=72,
                closed=True,
            ),
            "juicer_bowl_shell",
        ),
        material="brushed_steel",
        name="bowl_shell",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.085),
        origin=Origin(xyz=(0.0, -0.186, 0.218), rpy=(pi / 2.0, 0.0, 0.0)),
        material="brushed_steel",
        name="juice_spout",
    )
    body.visual(Box((0.055, 0.230, 0.160)), origin=Origin(xyz=(0.176, 0.000, 0.185)), material="warm_white", name="pulp_channel_housing")
    body.visual(Box((0.005, 0.195, 0.135)), origin=Origin(xyz=(0.206, 0.000, 0.195)), material="soft_gray", name="service_door_recess")
    body.visual(Box((0.004, 0.205, 0.016)), origin=Origin(xyz=(0.209, 0.000, 0.263)), material="dark_gray", name="door_top_gasket")
    body.visual(Box((0.004, 0.205, 0.016)), origin=Origin(xyz=(0.209, 0.000, 0.127)), material="dark_gray", name="door_bottom_gasket")
    body.visual(Box((0.004, 0.016, 0.135)), origin=Origin(xyz=(0.209, -0.101, 0.195)), material="dark_gray", name="door_front_gasket")
    body.visual(Box((0.004, 0.016, 0.135)), origin=Origin(xyz=(0.209, 0.101, 0.195)), material="dark_gray", name="door_rear_gasket")
    body.visual(
        Cylinder(radius=0.008, length=0.035),
        origin=Origin(xyz=(0.211, 0.095, 0.145)),
        material="brushed_steel",
        name="side_hinge_lower",
    )
    body.visual(
        Cylinder(radius=0.008, length=0.035),
        origin=Origin(xyz=(0.211, 0.095, 0.245)),
        material="brushed_steel",
        name="side_hinge_upper",
    )
    body.visual(Box((0.004, 0.090, 0.040)), origin=Origin(xyz=(0.192, 0.000, 0.075)), material="dark_gray", name="control_panel")
    body.visual(
        Cylinder(radius=0.009, length=0.050),
        origin=Origin(xyz=(-0.115, 0.148, 0.279), rpy=(0.0, pi / 2.0, 0.0)),
        material="brushed_steel",
        name="rear_hinge_knuckle_0",
    )
    body.visual(
        Cylinder(radius=0.009, length=0.050),
        origin=Origin(xyz=(0.115, 0.148, 0.279), rpy=(0.0, pi / 2.0, 0.0)),
        material="brushed_steel",
        name="rear_hinge_knuckle_1",
    )
    body.visual(Cylinder(radius=0.022, length=0.018), origin=Origin(xyz=(0.0, 0.0, 0.151)), material="dark_gray", name="basket_drive_coupler")
    body.visual(Box((0.210, 0.018, 0.030)), origin=Origin(xyz=(0.0, 0.151, 0.255)), material="soft_gray", name="rear_hinge_mount")

    basket = model.part("filter_basket")
    basket.visual(mesh_from_geometry(_merged_filter_basket(), "filter_basket"), material="brushed_steel", name="perforated_basket")
    model.articulation(
        "body_to_filter_basket",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=basket,
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=40.0),
    )

    lid = model.part("lid")
    lid.visual(mesh_from_geometry(_merged_lid(), "transparent_lid"), material="smoked_clear", name="clear_lid_shell")
    lid.visual(Box((0.110, 0.025, 0.006)), origin=Origin(xyz=(0.0, -0.010, 0.003)), material="smoked_clear", name="rear_hinge_leaf")
    lid.visual(
        Cylinder(radius=0.007, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="smoked_clear",
        name="rear_hinge_barrel",
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.148, 0.280)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.35),
    )

    pusher = model.part("feed_pusher")
    pusher.visual(Cylinder(radius=0.030, length=0.190), origin=Origin(xyz=(0.0, 0.0, -0.085)), material="pusher_frosted", name="pusher_cylinder")
    pusher.visual(Cylinder(radius=0.044, length=0.026), origin=Origin(xyz=(0.0, 0.0, 0.013)), material="pusher_frosted", name="pusher_cap")
    model.articulation(
        "lid_to_feed_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.0, -0.150, 0.176)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.30, lower=0.0, upper=0.115),
    )

    door = model.part("service_door")
    door.visual(Cylinder(radius=0.006, length=0.065), origin=Origin(xyz=(0.0, 0.0, 0.0)), material="brushed_steel", name="door_hinge_barrel")
    door.visual(Box((0.012, 0.022, 0.115)), origin=Origin(xyz=(0.010, -0.006, 0.0)), material="soft_gray", name="door_hinge_leaf")
    door.visual(Box((0.014, 0.160, 0.120)), origin=Origin(xyz=(0.012, -0.095, 0.0)), material="warm_white", name="door_panel")
    door.visual(Box((0.004, 0.080, 0.012)), origin=Origin(xyz=(0.020, -0.095, 0.030)), material="dark_gray", name="door_grip")
    model.articulation(
        "body_to_service_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.211, 0.095, 0.195)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.25),
    )

    knob = model.part("control_knob")
    knob.visual(
        Cylinder(radius=0.006, length=0.016),
        origin=Origin(xyz=(0.007, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="brushed_steel",
        name="knob_shaft",
    )
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.052,
                0.026,
                body_style="skirted",
                top_diameter=0.040,
                skirt=KnobSkirt(0.058, 0.005, flare=0.05, chamfer=0.001),
                grip=KnobGrip(style="fluted", count=18, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "control_knob_cap",
        ),
        origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="dark_gray",
        name="knob_cap",
    )
    model.articulation(
        "body_to_control_knob",
        ArticulationType.REVOLUTE,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.195, 0.000, 0.075)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0, lower=-2.35, upper=2.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("feed_pusher")
    basket = object_model.get_part("filter_basket")
    door = object_model.get_part("service_door")
    knob = object_model.get_part("control_knob")

    lid_hinge = object_model.get_articulation("body_to_lid")
    pusher_slide = object_model.get_articulation("lid_to_feed_pusher")
    door_hinge = object_model.get_articulation("body_to_service_door")
    knob_turn = object_model.get_articulation("body_to_control_knob")

    def _aabb_max_z(aabb):
        return None if aabb is None else aabb[1][2]

    def _aabb_center_x(aabb):
        return None if aabb is None else 0.5 * (aabb[0][0] + aabb[1][0])

    with ctx.pose({lid_hinge: 0.0, pusher_slide: 0.0}):
        ctx.expect_within(pusher, lid, axes="xy", inner_elem="pusher_cylinder", outer_elem="clear_lid_shell", margin=0.004, name="pusher is centered in the lid chute")
        ctx.expect_overlap(pusher, lid, axes="z", elem_a="pusher_cylinder", elem_b="clear_lid_shell", min_overlap=0.12, name="pusher remains guided through the chute")
        ctx.expect_contact(pusher, lid, elem_a="pusher_cap", elem_b="clear_lid_shell", contact_tol=0.002, name="pusher cap rests on the chute rim")
        ctx.expect_within(basket, body, axes="xy", inner_elem="perforated_basket", outer_elem="bowl_shell", margin=0.003, name="filter basket sits inside the bowl shell")
        ctx.expect_gap(lid, basket, axis="z", min_gap=0.010, name="closed lid clears the spinning filter basket")

    rest_pusher = ctx.part_world_position(pusher)
    with ctx.pose({pusher_slide: 0.100}):
        lifted_pusher = ctx.part_world_position(pusher)
        ctx.expect_overlap(pusher, lid, axes="z", elem_a="pusher_cylinder", elem_b="clear_lid_shell", min_overlap=0.050, name="raised pusher is still retained in the chute")
    ctx.check(
        "pusher slides upward",
        rest_pusher is not None and lifted_pusher is not None and lifted_pusher[2] > rest_pusher[2] + 0.080,
        details=f"rest={rest_pusher}, lifted={lifted_pusher}",
    )

    with ctx.pose({lid_hinge: 0.0}):
        rest_lid_shell = ctx.part_element_world_aabb(lid, elem="clear_lid_shell")
    with ctx.pose({lid_hinge: 1.0}):
        raised_lid_shell = ctx.part_element_world_aabb(lid, elem="clear_lid_shell")
    ctx.check(
        "lid hinge opens upward",
        _aabb_max_z(rest_lid_shell) is not None and _aabb_max_z(raised_lid_shell) is not None and _aabb_max_z(raised_lid_shell) > _aabb_max_z(rest_lid_shell) + 0.070,
        details=f"rest={rest_lid_shell}, raised={raised_lid_shell}",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(door, body, axis="x", min_gap=0.001, positive_elem="door_panel", negative_elem="service_door_recess", name="closed service door sits outside body side")
        closed_door_panel = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_hinge: 1.0}):
        swung_door_panel = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "service door swings outward",
        _aabb_center_x(closed_door_panel) is not None and _aabb_center_x(swung_door_panel) is not None and _aabb_center_x(swung_door_panel) > _aabb_center_x(closed_door_panel) + 0.045,
        details=f"closed={closed_door_panel}, swung={swung_door_panel}",
    )

    with ctx.pose({knob_turn: 1.2}):
        ctx.expect_gap(knob, body, axis="x", min_gap=0.002, positive_elem="knob_cap", negative_elem="control_panel", name="control knob remains proud of the side panel")

    return ctx.report()


object_model = build_object_model()
