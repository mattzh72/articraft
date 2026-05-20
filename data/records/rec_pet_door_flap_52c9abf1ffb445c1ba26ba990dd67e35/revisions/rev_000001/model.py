from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="insulated_double_flap_pet_door")

    dark_plastic = model.material("dark_graphite_plastic", rgba=(0.04, 0.045, 0.05, 1.0))
    rubber = model.material("black_rubber_weatherstrip", rgba=(0.005, 0.005, 0.004, 1.0))
    smoked_panel = model.material("smoked_polycarbonate", rgba=(0.42, 0.58, 0.66, 0.55))
    foam_edge = model.material("warm_gray_insulated_edge", rgba=(0.60, 0.62, 0.58, 1.0))
    magnet = model.material("dark_magnetic_strip", rgba=(0.015, 0.017, 0.018, 1.0))
    hinge_mat = model.material("satin_steel_hinge_pins", rgba=(0.62, 0.64, 0.62, 1.0))

    opening_w = 0.34
    opening_h = 0.49
    outer_w = 0.52
    outer_h = 0.69
    frame_depth = 0.14

    frame = model.part("frame")
    frame.visual(
        mesh_from_geometry(
            BezelGeometry(
                opening_size=(opening_w, opening_h),
                outer_size=(outer_w, outer_h),
                depth=frame_depth,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.045,
                outer_corner_radius=0.060,
            ),
            "rounded_double_flap_frame",
        ),
        # Bezel depth is local Z; rotate it so the tube passes through the door
        # thickness along world Y while the opening remains X by Z.
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="rounded_frame_tube",
    )

    # Soft weather-stripping on both lips shows the insulated air pocket and
    # gives each rigid flap a real sealing surface without touching it at rest.
    seal_y = frame_depth / 2.0 - 0.001
    for side_name, y in (("exterior", -seal_y), ("interior", seal_y)):
        frame.visual(
            Box((0.012, 0.006, 0.455)),
            origin=Origin(xyz=(-0.166, y, 0.0)),
            material=rubber,
            name=f"{side_name}_seal_side_0",
        )
        frame.visual(
            Box((0.012, 0.006, 0.455)),
            origin=Origin(xyz=(0.166, y, 0.0)),
            material=rubber,
            name=f"{side_name}_seal_side_1",
        )
        frame.visual(
            Box((0.305, 0.006, 0.012)),
            origin=Origin(xyz=(0.0, y, 0.241)),
            material=rubber,
            name=f"{side_name}_seal_top",
        )
        frame.visual(
            Box((0.305, 0.006, 0.012)),
            origin=Origin(xyz=(0.0, y, -0.241)),
            material=rubber,
            name=f"{side_name}_seal_bottom",
        )

    # Hinge cheeks are fixed to the outer frame at each side of each flap.
    # They sit outside the clear opening, leaving the child hinge barrels free.
    hinge_z = 0.222
    exterior_y = -(frame_depth / 2.0 + 0.013)
    interior_y = frame_depth / 2.0 + 0.013
    for side_name, y in (("exterior", exterior_y), ("interior", interior_y)):
        for idx, x in enumerate((-0.183, 0.183)):
            frame.visual(
                Box((0.026, 0.030, 0.044)),
                origin=Origin(xyz=(x, y, hinge_z)),
                material=dark_plastic,
                name=f"{side_name}_hinge_cheek_{idx}",
            )
            frame.visual(
                Cylinder(radius=0.0045, length=0.040),
                origin=Origin(xyz=(0.158 if x > 0.0 else -0.158, y, hinge_z), rpy=(0.0, pi / 2.0, 0.0)),
                material=hinge_mat,
                name=f"{side_name}_pin_stub_{idx}",
            )

    flap_w = 0.305
    flap_h = 0.430
    panel_t = 0.010

    def add_flap(name: str, y: float, tint: Material) -> object:
        flap = model.part(name)
        flap.visual(
            Box((flap_w, panel_t, flap_h - 0.020)),
            # The child part frame is the hinge axis; the rigid panel hangs below
            # the barrel so the fixed pin passes through the hinge sleeve, not
            # through the clear panel.
            origin=Origin(xyz=(0.0, 0.0, -(flap_h / 2.0 + 0.010))),
            material=tint,
            name="rigid_panel",
        )
        flap.visual(
            Cylinder(radius=0.011, length=flap_w - 0.018),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=foam_edge,
            name="top_hinge_barrel",
        )
        flap.visual(
            Box((flap_w - 0.050, 0.014, 0.018)),
            origin=Origin(xyz=(0.0, 0.002, -0.012)),
            material=foam_edge,
            name="top_insulated_cap",
        )
        flap.visual(
            Box((0.017, 0.014, flap_h - 0.028)),
            origin=Origin(xyz=(-(flap_w / 2.0 + 0.002), 0.002, -flap_h / 2.0 - 0.006)),
            material=foam_edge,
            name="side_edge_0",
        )
        flap.visual(
            Box((0.017, 0.014, flap_h - 0.028)),
            origin=Origin(xyz=((flap_w / 2.0 + 0.002), 0.002, -flap_h / 2.0 - 0.006)),
            material=foam_edge,
            name="side_edge_1",
        )
        flap.visual(
            Box((flap_w + 0.010, 0.014, 0.022)),
            origin=Origin(xyz=(0.0, 0.002, -flap_h + 0.010)),
            material=foam_edge,
            name="bottom_insulated_edge",
        )
        flap.visual(
            Box((flap_w - 0.035, 0.016, 0.014)),
            origin=Origin(xyz=(0.0, -0.001, -flap_h + 0.023)),
            material=magnet,
            name="magnetic_closer",
        )
        return flap

    exterior_flap = add_flap("exterior_flap", exterior_y, smoked_panel)
    interior_flap = add_flap("interior_flap", interior_y, smoked_panel)

    bidirectional_limits = MotionLimits(effort=4.0, velocity=2.5, lower=-1.10, upper=1.10)
    model.articulation(
        "exterior_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=exterior_flap,
        origin=Origin(xyz=(0.0, exterior_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=bidirectional_limits,
    )
    model.articulation(
        "interior_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=interior_flap,
        origin=Origin(xyz=(0.0, interior_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=bidirectional_limits,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    exterior_flap = object_model.get_part("exterior_flap")
    interior_flap = object_model.get_part("interior_flap")
    exterior_hinge = object_model.get_articulation("exterior_hinge")
    interior_hinge = object_model.get_articulation("interior_hinge")

    ctx.check(
        "two separate flap hinges",
        exterior_hinge.child == "exterior_flap"
        and interior_hinge.child == "interior_flap"
        and exterior_hinge.parent == "frame"
        and interior_hinge.parent == "frame",
        details="Both rigid flaps should be independently hinged from the fixed frame.",
    )
    ctx.check(
        "hinge axes are horizontal and separated",
        exterior_hinge.axis == (1.0, 0.0, 0.0)
        and interior_hinge.axis == (1.0, 0.0, 0.0)
        and exterior_hinge.origin.xyz[1] < -0.07
        and interior_hinge.origin.xyz[1] > 0.07
        and abs(exterior_hinge.origin.xyz[2] - interior_hinge.origin.xyz[2]) < 1e-6,
        details=f"exterior={exterior_hinge.origin}, interior={interior_hinge.origin}",
    )
    ctx.check(
        "both flaps swing bidirectionally",
        exterior_hinge.motion_limits.lower <= -1.0
        and exterior_hinge.motion_limits.upper >= 1.0
        and interior_hinge.motion_limits.lower <= -1.0
        and interior_hinge.motion_limits.upper >= 1.0,
        details="A pet door flap should be able to swing inward or outward from its top hinge.",
    )

    ctx.expect_gap(
        interior_flap,
        exterior_flap,
        axis="y",
        min_gap=0.13,
        max_gap=0.16,
        name="closed flaps leave insulating air gap",
    )
    ctx.expect_overlap(
        exterior_flap,
        interior_flap,
        axes="xz",
        min_overlap=0.25,
        name="double flaps cover the same pet opening",
    )

    for flap, side in ((exterior_flap, "exterior"), (interior_flap, "interior")):
        for idx in (0, 1):
            pin = f"{side}_pin_stub_{idx}"
            ctx.allow_overlap(
                frame,
                flap,
                elem_a=pin,
                elem_b="top_hinge_barrel",
                reason="The fixed hinge pin is intentionally captured inside the flap's hinge barrel.",
            )
            ctx.expect_within(
                frame,
                flap,
                axes="yz",
                inner_elem=pin,
                outer_elem="top_hinge_barrel",
                margin=0.0005,
                name=f"{pin} sits inside hinge barrel cross-section",
            )
            ctx.expect_overlap(
                frame,
                flap,
                axes="x",
                elem_a=pin,
                elem_b="top_hinge_barrel",
                min_overlap=0.004,
                name=f"{pin} retains hinge barrel axially",
            )

    exterior_rest = ctx.part_element_world_aabb(exterior_flap, elem="rigid_panel")
    interior_rest = ctx.part_element_world_aabb(interior_flap, elem="rigid_panel")
    with ctx.pose({exterior_hinge: 0.75}):
        exterior_open = ctx.part_element_world_aabb(exterior_flap, elem="rigid_panel")
        interior_still = ctx.part_element_world_aabb(interior_flap, elem="rigid_panel")
    with ctx.pose({interior_hinge: -0.75}):
        interior_open = ctx.part_element_world_aabb(interior_flap, elem="rigid_panel")

    ctx.check(
        "exterior flap moves independently",
        exterior_rest is not None
        and exterior_open is not None
        and interior_rest is not None
        and interior_still is not None
        and exterior_open[1][1] > exterior_rest[1][1] + 0.18
        and abs(interior_still[0][1] - interior_rest[0][1]) < 1e-6
        and abs(interior_still[1][1] - interior_rest[1][1]) < 1e-6,
        details=f"rest={exterior_rest}, open={exterior_open}, interior_rest={interior_rest}, interior_still={interior_still}",
    )
    ctx.check(
        "interior flap moves independently",
        interior_rest is not None
        and interior_open is not None
        and interior_open[0][1] < interior_rest[0][1] - 0.18,
        details=f"rest={interior_rest}, open={interior_open}",
    )

    return ctx.report()


object_model = build_object_model()
