from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelFace,
    BezelGeometry,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_toaster_oven")

    matte_graphite = model.material("matte_graphite", rgba=(0.18, 0.18, 0.17, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.62, 0.61, 0.57, 1.0))
    satin_black = model.material("satin_black", rgba=(0.035, 0.037, 0.038, 1.0))
    seam_black = model.material("seam_black", rgba=(0.006, 0.006, 0.006, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.08, 0.11, 0.13, 0.42))
    warm_coil = model.material("warm_coil", rgba=(1.0, 0.32, 0.08, 1.0))
    porcelain = model.material("porcelain_bushings", rgba=(0.82, 0.78, 0.68, 1.0))
    indicator_red = model.material("indicator_red", rgba=(0.85, 0.04, 0.025, 1.0))

    body = model.part("oven_body")

    # Rectangular, hollow appliance body: separate shell panels preserve a real
    # open oven cavity instead of a single solid block.
    body.visual(
        Box((0.56, 0.38, 0.035)),
        origin=Origin(xyz=(0.0, 0.010, 0.3225)),
        material=matte_graphite,
        name="top_shell",
    )
    body.visual(
        Box((0.56, 0.38, 0.035)),
        origin=Origin(xyz=(0.0, 0.010, 0.0375)),
        material=matte_graphite,
        name="bottom_shell",
    )
    body.visual(
        Box((0.035, 0.38, 0.285)),
        origin=Origin(xyz=(-0.2625, 0.010, 0.1775)),
        material=matte_graphite,
        name="side_wall_0",
    )
    body.visual(
        Box((0.035, 0.38, 0.285)),
        origin=Origin(xyz=(0.2625, 0.010, 0.1775)),
        material=matte_graphite,
        name="side_wall_1",
    )
    body.visual(
        Box((0.56, 0.032, 0.285)),
        origin=Origin(xyz=(0.0, 0.184, 0.1775)),
        material=matte_graphite,
        name="rear_shell",
    )
    body.visual(
        Box((0.025, 0.38, 0.285)),
        origin=Origin(xyz=(0.105, 0.010, 0.1775)),
        material=matte_graphite,
        name="cavity_divider",
    )

    # Continuous front architecture: recessed cooking cavity bezel on the left
    # and a satin control fascia on the right, divided by crisp black seams.
    body.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.335, 0.188),
                (0.405, 0.255),
                0.012,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.012,
                outer_corner_radius=0.017,
                face=BezelFace(style="radiused_step", front_lip=0.0025, fillet=0.002),
            ),
            "oven_opening_bezel",
        ),
        origin=Origin(xyz=(-0.075, -0.190, 0.188), rpy=(pi / 2, 0.0, 0.0)),
        material=satin_black,
        name="oven_opening_bezel",
    )
    body.visual(
        Box((0.162, 0.020, 0.262)),
        origin=Origin(xyz=(0.186, -0.189, 0.184)),
        material=satin_steel,
        name="control_fascia",
    )
    body.visual(
        Box((0.006, 0.009, 0.220)),
        origin=Origin(xyz=(0.104, -0.203, 0.195)),
        material=seam_black,
        name="vertical_door_seam",
    )
    body.visual(
        Box((0.148, 0.009, 0.006)),
        origin=Origin(xyz=(0.186, -0.203, 0.309)),
        material=seam_black,
        name="fascia_top_seam",
    )
    body.visual(
        Box((0.148, 0.009, 0.006)),
        origin=Origin(xyz=(0.186, -0.203, 0.050)),
        material=seam_black,
        name="fascia_bottom_seam",
    )

    # Dark enamel cavity, rack rails, wire shelf, and heating elements visible
    # through the smoked glass.
    body.visual(
        Box((0.348, 0.012, 0.190)),
        origin=Origin(xyz=(-0.075, 0.164, 0.187)),
        material=satin_black,
        name="rear_enamel_wall",
    )
    body.visual(
        Box((0.345, 0.260, 0.010)),
        origin=Origin(xyz=(-0.075, 0.030, 0.075)),
        material=satin_black,
        name="enamel_floor",
    )
    for x in (-0.238, 0.091):
        body.visual(
            Box((0.022, 0.245, 0.014)),
            origin=Origin(xyz=(x, 0.025, 0.152)),
            material=satin_steel,
            name=f"rack_side_rail_{0 if x < 0 else 1}",
        )
    for idx, x in enumerate((-0.205, -0.155, -0.105, -0.055, -0.005, 0.045)):
        body.visual(
            Box((0.006, 0.235, 0.006)),
            origin=Origin(xyz=(x, 0.025, 0.156)),
            material=satin_steel,
            name=f"rack_long_wire_{idx}",
        )
    for idx, y in enumerate((-0.080, -0.025, 0.030, 0.085, 0.140)):
        body.visual(
            Box((0.350, 0.006, 0.006)),
            origin=Origin(xyz=(-0.075, y, 0.158)),
            material=satin_steel,
            name=f"rack_cross_wire_{idx}",
        )
    for idx, (z, y) in enumerate(((0.104, -0.004), (0.262, -0.002))):
        body.visual(
            Cylinder(radius=0.006, length=0.345),
            origin=Origin(xyz=(-0.075, y, z), rpy=(0.0, pi / 2, 0.0)),
            material=warm_coil,
            name=f"heating_element_{idx}",
        )
        for x in (-0.238, 0.091):
            body.visual(
                Box((0.020, 0.018, 0.018)),
                origin=Origin(xyz=(x, y, z)),
                material=porcelain,
                name=f"heater_bushing_{idx}_{0 if x < 0 else 1}",
            )

    # Slotted top vent, subtly inset in the matte shell.
    body.visual(
        mesh_from_geometry(
            SlotPatternPanelGeometry(
                (0.240, 0.090),
                0.003,
                slot_size=(0.070, 0.006),
                pitch=(0.082, 0.018),
                frame=0.010,
                corner_radius=0.007,
                stagger=True,
            ),
            "top_vent_panel",
        ),
        origin=Origin(xyz=(-0.040, -0.004, 0.341)),
        material=satin_black,
        name="top_vent_panel",
    )

    # Four low, shadowed feet attached to the bottom pan.
    for idx, (x, y) in enumerate(((-0.215, -0.122), (0.215, -0.122), (-0.215, 0.145), (0.215, 0.145))):
        body.visual(
            Box((0.072, 0.042, 0.020)),
            origin=Origin(xyz=(x, y, 0.010)),
            material=satin_black,
            name=f"foot_{idx}",
        )

    # Real bottom hinge sockets mounted to the lower sill; the door carries a
    # matching pivot pin on the same axis.
    for idx, x in enumerate((-0.290, 0.135)):
        body.visual(
            Box((0.048, 0.026, 0.026)),
            origin=Origin(xyz=(x, -0.196, 0.041)),
            material=satin_steel,
            name=f"hinge_bracket_{idx}",
        )
        body.visual(
            Cylinder(radius=0.011, length=0.046),
            origin=Origin(xyz=(x, -0.205, 0.065), rpy=(0.0, pi / 2, 0.0)),
            material=satin_steel,
            name=f"hinge_socket_{idx}",
        )

    # Refined fascia details: restrained display window, status light, and
    # printed/tactile tick marks around each rotary control.
    body.visual(
        Box((0.078, 0.004, 0.026)),
        origin=Origin(xyz=(0.186, -0.201, 0.287)),
        material=satin_black,
        name="display_window",
    )
    body.visual(
        Cylinder(radius=0.0065, length=0.004),
        origin=Origin(xyz=(0.240, -0.201, 0.287), rpy=(pi / 2, 0.0, 0.0)),
        material=indicator_red,
        name="status_light",
    )
    knob_centers = (0.244, 0.179, 0.114)
    for row, zc in enumerate(knob_centers):
        body.visual(
            Cylinder(radius=0.035, length=0.003),
            origin=Origin(xyz=(0.186, -0.2005, zc), rpy=(pi / 2, 0.0, 0.0)),
            material=satin_black,
            name=f"dial_recess_{row}",
        )
        for idx, (dx, dz, sx, sz) in enumerate(
            (
                (0.000, 0.046, 0.004, 0.014),
                (0.000, -0.046, 0.004, 0.014),
                (-0.047, 0.000, 0.014, 0.004),
                (0.047, 0.000, 0.014, 0.004),
                (-0.034, 0.034, 0.010, 0.004),
                (0.034, 0.034, 0.010, 0.004),
            )
        ):
            body.visual(
                Box((sx, 0.003, sz)),
                origin=Origin(xyz=(0.186 + dx, -0.2005, zc + dz)),
                material=seam_black,
                name=f"dial_tick_{row}_{idx}",
            )

    door = model.part("door")
    door.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.286, 0.158),
                (0.374, 0.236),
                0.018,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.010,
                outer_corner_radius=0.018,
                face=BezelFace(style="radiused_step", front_lip=0.002, fillet=0.002),
            ),
            "door_frame",
        ),
        origin=Origin(xyz=(0.0, -0.010, 0.120), rpy=(pi / 2, 0.0, 0.0)),
        material=satin_steel,
        name="door_frame",
    )
    door.visual(
        Box((0.306, 0.006, 0.176)),
        origin=Origin(xyz=(0.0, -0.012, 0.132)),
        material=smoked_glass,
        name="glass_pane",
    )
    door.visual(
        Cylinder(radius=0.008, length=0.460),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
        material=satin_steel,
        name="door_pivot",
    )
    door.visual(
        Cylinder(radius=0.009, length=0.262),
        origin=Origin(xyz=(0.0, -0.048, 0.213), rpy=(0.0, pi / 2, 0.0)),
        material=satin_steel,
        name="front_handle",
    )
    for idx, x in enumerate((-0.108, 0.108)):
        door.visual(
            Cylinder(radius=0.006, length=0.036),
            origin=Origin(xyz=(x, -0.030, 0.213), rpy=(-pi / 2, 0.0, 0.0)),
            material=satin_steel,
            name=f"handle_standoff_{idx}",
        )

    door_joint = model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-0.075, -0.205, 0.065)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=1.45),
    )
    door_joint.meta["qc_samples"] = [0.0, 0.75, 1.35]

    knob_meshes = []
    for name in ("temp_knob_mesh", "mode_knob_mesh", "timer_knob_mesh"):
        knob_meshes.append(
            mesh_from_geometry(
                KnobGeometry(
                    0.047,
                    0.026,
                    body_style="skirted",
                    top_diameter=0.036,
                    skirt=KnobSkirt(0.055, 0.006, flare=0.06, chamfer=0.0012),
                    grip=KnobGrip(style="fluted", count=22, depth=0.0012),
                    indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008, angle_deg=90.0),
                    edge_radius=0.001,
                    center=False,
                ),
                name,
            )
        )

    for link_name, mesh, zc, upper in (
        ("temp_knob", knob_meshes[0], 0.244, 4.70),
        ("mode_knob", knob_meshes[1], 0.179, 3.70),
        ("timer_knob", knob_meshes[2], 0.114, 5.20),
    ):
        knob = model.part(link_name)
        knob.visual(
            mesh,
            origin=Origin(),
            material=satin_steel,
            name="knob_cap",
        )
        joint = model.articulation(
            f"body_to_{link_name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(xyz=(0.186, -0.202, zc), rpy=(pi / 2, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=1.5, velocity=3.0, lower=0.0, upper=upper),
        )
        joint.meta["qc_samples"] = [0.0, upper * 0.55]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("oven_body")
    door = object_model.get_part("door")
    door_joint = object_model.get_articulation("body_to_door")

    for idx in (0, 1):
        ctx.allow_overlap(
            body,
            door,
            elem_a=f"hinge_socket_{idx}",
            elem_b="door_pivot",
            reason="The door pivot pin is intentionally captured inside the satin hinge socket.",
        )
        ctx.expect_overlap(
            body,
            door,
            axes="x",
            elem_a=f"hinge_socket_{idx}",
            elem_b="door_pivot",
            min_overlap=0.020,
            name=f"hinge socket {idx} captures the door pivot",
        )

    with ctx.pose({door_joint: 0.0}):
        ctx.expect_gap(
            body,
            door,
            axis="y",
            positive_elem="oven_opening_bezel",
            negative_elem="door_frame",
            min_gap=0.004,
            max_gap=0.020,
            name="closed door sits just proud of the front bezel",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            elem_a="glass_pane",
            elem_b="oven_opening_bezel",
            min_overlap=0.135,
            name="viewing glass is framed by the oven opening",
        )

        closed_handle_aabb = ctx.part_element_world_aabb(door, elem="front_handle")

    with ctx.pose({door_joint: 1.20}):
        opened_handle_aabb = ctx.part_element_world_aabb(door, elem="front_handle")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    closed_handle = _aabb_center(closed_handle_aabb)
    opened_handle = _aabb_center(opened_handle_aabb)

    ctx.check(
        "door rotates outward and down on the bottom hinge",
        closed_handle is not None
        and opened_handle is not None
        and opened_handle[1] < closed_handle[1] - 0.10
        and opened_handle[2] < closed_handle[2] - 0.10,
        details=f"closed_handle={closed_handle}, opened_handle={opened_handle}",
    )

    for link_name in ("temp_knob", "mode_knob", "timer_knob"):
        knob = object_model.get_part(link_name)
        ctx.expect_contact(
            knob,
            body,
            elem_a="knob_cap",
            elem_b="control_fascia",
            contact_tol=0.006,
            name=f"{link_name} is seated on the fascia",
        )

    return ctx.report()


object_model = build_object_model()
