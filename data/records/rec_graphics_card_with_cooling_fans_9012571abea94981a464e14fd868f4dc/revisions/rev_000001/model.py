from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="triple_fan_graphics_card")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    satin_black = model.material("satin_black", rgba=(0.035, 0.038, 0.043, 1.0))
    pcb_black = model.material("pcb_black", rgba=(0.005, 0.030, 0.025, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.010, 0.011, 0.012, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.22, 0.24, 0.26, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.72, 0.73, 0.70, 1.0))
    fin_metal = model.material("fin_metal", rgba=(0.64, 0.66, 0.68, 1.0))
    copper = model.material("copper", rgba=(0.75, 0.34, 0.10, 1.0))
    gold = model.material("gold_contacts", rgba=(1.00, 0.73, 0.20, 1.0))
    port_black = model.material("port_black", rgba=(0.002, 0.002, 0.002, 1.0))
    screw_metal = model.material("screw_metal", rgba=(0.10, 0.11, 0.12, 1.0))

    body = model.part("card_body")

    # A full-length PCB and backplate establish the thin electronics component
    # proportions; the fan shroud and heatsink are built up on the +Z face.
    body.visual(
        Box((0.330, 0.116, 0.004)),
        origin=Origin(xyz=(0.000, 0.000, -0.006)),
        material=pcb_black,
        name="pcb",
    )
    body.visual(
        Box((0.315, 0.106, 0.003)),
        origin=Origin(xyz=(0.008, 0.000, -0.0093)),
        material=gunmetal,
        name="backplate",
    )
    body.visual(
        Box((0.268, 0.092, 0.016)),
        origin=Origin(xyz=(0.014, 0.000, 0.004)),
        material=fin_metal,
        name="heatsink_base",
    )

    # Dense aluminum fin stack visible through the fan openings.  The fins sit
    # on the base plate and are tied together by the base, so they read as a
    # continuous cooler instead of separate floating plates.
    for i, x in enumerate([x * 0.016 - 0.120 for x in range(16)]):
        body.visual(
            Box((0.0024, 0.088, 0.018)),
            origin=Origin(xyz=(x, 0.000, 0.013)),
            material=fin_metal,
            name=f"cooling_fin_{i}",
        )

    # Copper heat pipes routed through the fin stack.
    for i, y in enumerate((-0.030, -0.010, 0.010, 0.030)):
        body.visual(
            Cylinder(radius=0.0032, length=0.250),
            origin=Origin(xyz=(0.015, y, 0.015), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=copper,
            name=f"heat_pipe_{i}",
        )

    # Long, open fan shroud: rails, end caps and separators form real openings
    # rather than a solid slab behind the fans.
    body.visual(
        Box((0.292, 0.024, 0.018)),
        origin=Origin(xyz=(0.015, 0.055, 0.021)),
        material=matte_black,
        name="top_shroud_rail",
    )
    body.visual(
        Box((0.292, 0.024, 0.018)),
        origin=Origin(xyz=(0.015, -0.055, 0.021)),
        material=matte_black,
        name="bottom_shroud_rail",
    )
    body.visual(
        Box((0.018, 0.116, 0.018)),
        origin=Origin(xyz=(-0.142, 0.000, 0.021)),
        material=matte_black,
        name="rear_shroud_cap",
    )
    body.visual(
        Box((0.018, 0.116, 0.018)),
        origin=Origin(xyz=(0.172, 0.000, 0.021)),
        material=matte_black,
        name="front_shroud_cap",
    )
    for i, x in enumerate((-0.047, 0.077)):
        body.visual(
            Box((0.010, 0.108, 0.018)),
            origin=Origin(xyz=(x, 0.000, 0.021)),
            material=matte_black,
            name=f"shroud_spine_{i}",
        )

    fan_centers = (-0.096, 0.015, 0.126)
    for i, x in enumerate(fan_centers):
        body.visual(
            mesh_from_geometry(
                TorusGeometry(radius=0.0395, tube=0.0046, radial_segments=36, tubular_segments=12),
                f"fan_bezel_{i}",
            ),
            origin=Origin(xyz=(x, 0.000, 0.028)),
            material=satin_black,
            name=f"fan_bezel_{i}",
        )
        body.visual(
            Cylinder(radius=0.0125, length=0.006),
            origin=Origin(xyz=(x, 0.000, 0.020)),
            material=dark_plastic,
            name=f"fan_motor_{i}",
        )
        # Three stator spokes attach the fixed motor boss to the circular
        # shroud ring and are visibly behind the moving blades.
        for j, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
            body.visual(
                Box((0.030, 0.0032, 0.0032)),
                origin=Origin(
                    xyz=(x + math.cos(angle) * 0.024, math.sin(angle) * 0.024, 0.020),
                    rpy=(0.0, 0.0, angle),
                ),
                material=dark_plastic,
                name=f"fan_spoke_{i}_{j}",
            )
    # Raised angular accent ribs give the shroud a high-performance GPU look.
    for i, (x, y, yaw) in enumerate(
        (
            (-0.120, 0.033, 0.42),
            (-0.061, -0.034, -0.42),
            (0.050, 0.034, 0.42),
            (0.111, -0.034, -0.42),
        )
    ):
        body.visual(
            Box((0.060, 0.0055, 0.004)),
            origin=Origin(xyz=(x, y, 0.032), rpy=(0.0, 0.0, yaw)),
            material=gunmetal,
            name=f"accent_rib_{i}",
        )

    # Corner fasteners and molded bosses.
    for i, (x, y) in enumerate(((-0.137, 0.050), (-0.137, -0.050), (0.166, 0.050), (0.166, -0.050))):
        body.visual(
            Cylinder(radius=0.0055, length=0.004),
            origin=Origin(xyz=(x, y, 0.032)),
            material=screw_metal,
            name=f"shroud_screw_{i}",
        )

    # PCIe card-edge connector with individual gold fingers along the bottom edge.
    body.visual(
        Box((0.190, 0.014, 0.003)),
        origin=Origin(xyz=(-0.006, -0.064, -0.006)),
        material=gold,
        name="pcie_contact_bar",
    )
    for i in range(18):
        body.visual(
            Box((0.006, 0.0065, 0.0035)),
            origin=Origin(xyz=(-0.091 + i * 0.010, -0.071, -0.004)),
            material=gold,
            name=f"pcie_finger_{i}",
        )

    # Eight-pin auxiliary power socket on the top edge.
    body.visual(
        Box((0.035, 0.018, 0.018)),
        origin=Origin(xyz=(0.104, 0.068, 0.006)),
        material=port_black,
        name="power_socket",
    )
    for row, z in enumerate((0.003, 0.010)):
        for col, x in enumerate((0.094, 0.101, 0.108, 0.115)):
            body.visual(
                Box((0.0035, 0.002, 0.0035)),
                origin=Origin(xyz=(x, 0.078, z)),
                material=gunmetal,
                name=f"power_pin_{row}_{col}",
            )

    # Rear I/O bracket with dark port recesses, screw ears and a folded lip.
    body.visual(
        Box((0.006, 0.136, 0.082)),
        origin=Origin(xyz=(-0.168, 0.000, 0.020)),
        material=brushed_metal,
        name="io_bracket",
    )
    body.visual(
        Box((0.012, 0.118, 0.010)),
        origin=Origin(xyz=(-0.162, 0.000, -0.017)),
        material=brushed_metal,
        name="bracket_foot",
    )
    body.visual(
        Box((0.006, 0.028, 0.018)),
        origin=Origin(xyz=(-0.168, 0.081, 0.051)),
        material=brushed_metal,
        name="bracket_tab",
    )
    for i, (y, z, sy, sz) in enumerate(((-0.043, 0.030, 0.018, 0.011), (-0.010, 0.030, 0.020, 0.011), (0.026, 0.030, 0.020, 0.011), (0.047, 0.004, 0.030, 0.014))):
        body.visual(
            Box((0.003, sy, sz)),
            origin=Origin(xyz=(-0.1725, y, z)),
            material=port_black,
            name=f"display_port_{i}",
        )
    for i, (y, z) in enumerate(((0.060, 0.055), (0.060, -0.006))):
        body.visual(
            Cylinder(radius=0.004, length=0.003),
            origin=Origin(xyz=(-0.1725, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=port_black,
            name=f"bracket_hole_{i}",
        )

    # Three independent exposed axial fans.  Their part frames are on their
    # bearing axes so the continuous joints spin the blades in place.
    rotor_meshes = []
    for i in range(3):
        rotor_meshes.append(
            mesh_from_geometry(
                FanRotorGeometry(
                    outer_radius=0.034,
                    hub_radius=0.012,
                    blade_count=11,
                    thickness=0.009,
                    blade_pitch_deg=33.0,
                    blade_sweep_deg=29.0,
                    blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=14.0, camber=0.18, tip_clearance=0.0012),
                    hub=FanRotorHub(style="spinner", rear_collar_height=0.002, rear_collar_radius=0.010),
                ),
                f"fan_rotor_{i}",
            )
        )

    for i, x in enumerate(fan_centers):
        fan = model.part(f"fan_{i}")
        fan.visual(
            rotor_meshes[i],
            origin=Origin(),
            material=dark_plastic,
            name="rotor",
        )
        fan.visual(
            Cylinder(radius=0.0032, length=0.016),
            origin=Origin(xyz=(0.0, 0.0, -0.010)),
            material=screw_metal,
            name="shaft",
        )
        model.articulation(
            f"body_to_fan_{i}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=fan,
            origin=Origin(xyz=(x, 0.000, 0.034)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.12, velocity=180.0),
            motion_properties=MotionProperties(damping=0.002, friction=0.0005),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("card_body")
    fans = [object_model.get_part(f"fan_{i}") for i in range(3)]
    joints = [object_model.get_articulation(f"body_to_fan_{i}") for i in range(3)]

    # The model should read as a long, low graphics card rather than a generic
    # appliance block.
    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is not None:
        bmin, bmax = body_aabb
        ctx.check(
            "graphics card proportions are long and thin",
            (bmax[0] - bmin[0]) > 0.30 and (bmax[1] - bmin[1]) < 0.18 and (bmax[2] - bmin[2]) < 0.12,
            details=f"aabb={body_aabb}",
        )

    for i, (fan, joint) in enumerate(zip(fans, joints)):
        ctx.allow_overlap(
            fan,
            body,
            elem_a="shaft",
            elem_b=f"fan_motor_{i}",
            reason="The fan shaft is intentionally captured inside the fixed motor bushing.",
        )
        ctx.check(
            f"fan {i} uses continuous spin joint",
            joint.articulation_type == ArticulationType.CONTINUOUS and tuple(joint.axis) == (0.0, 0.0, 1.0),
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )
        ctx.expect_within(
            fan,
            body,
            axes="xy",
            inner_elem="shaft",
            outer_elem=f"fan_motor_{i}",
            margin=0.001,
            name=f"fan {i} shaft is centered in the motor bushing",
        )
        ctx.expect_overlap(
            fan,
            body,
            axes="z",
            elem_a="shaft",
            elem_b=f"fan_motor_{i}",
            min_overlap=0.004,
            name=f"fan {i} shaft remains inserted in the bushing",
        )
        ctx.expect_within(
            fan,
            body,
            axes="xy",
            inner_elem="rotor",
            outer_elem=f"fan_bezel_{i}",
            margin=0.003,
            name=f"fan {i} rotor sits inside its circular shroud bezel",
        )
        ctx.expect_gap(
            fan,
            body,
            axis="z",
            positive_elem="rotor",
            negative_elem=f"fan_motor_{i}",
            min_gap=0.004,
            max_gap=0.020,
            name=f"fan {i} rotor clears fixed motor boss",
        )
        rest = ctx.part_world_position(fan)
        with ctx.pose({joint: 1.25}):
            moved = ctx.part_world_position(fan)
            ctx.expect_within(
                fan,
                body,
                axes="xy",
                inner_elem="rotor",
                outer_elem=f"fan_bezel_{i}",
                margin=0.003,
                name=f"fan {i} remains centered while spinning",
            )
        ctx.check(
            f"fan {i} spin keeps bearing center fixed",
            rest is not None and moved is not None and abs(rest[0] - moved[0]) < 1e-6 and abs(rest[1] - moved[1]) < 1e-6,
            details=f"rest={rest}, moved={moved}",
        )

    ctx.expect_contact(
        body,
        body,
        elem_a="io_bracket",
        elem_b="pcb",
        contact_tol=0.002,
        name="metal I/O bracket is mounted to the PCB end",
    )

    return ctx.report()


object_model = build_object_model()
