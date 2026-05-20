from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drum_hand_truck")

    red = model.material("red_powdercoat", rgba=(0.78, 0.05, 0.035, 1.0))
    dark = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    steel = model.material("galvanized_steel", rgba=(0.55, 0.58, 0.60, 1.0))
    plate_gray = model.material("scuffed_scoop_steel", rgba=(0.35, 0.36, 0.34, 1.0))

    frame = model.part("frame")

    rail_points = {
        -1: [
            (-0.18, -0.235, 0.065),
            (-0.145, -0.235, 0.25),
            (-0.075, -0.235, 0.62),
            (-0.025, -0.235, 0.98),
            (0.025, -0.235, 1.25),
            (0.105, -0.235, 1.43),
        ],
        1: [
            (-0.18, 0.235, 0.065),
            (-0.145, 0.235, 0.25),
            (-0.075, 0.235, 0.62),
            (-0.025, 0.235, 0.98),
            (0.025, 0.235, 1.25),
            (0.105, 0.235, 1.43),
        ],
    }
    for idx, sign in enumerate((-1, 1)):
        frame.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    rail_points[sign],
                    radius=0.024,
                    samples_per_segment=14,
                    radial_segments=20,
                    cap_ends=True,
                ),
                f"curved_side_rail_{idx}",
            ),
            material=red,
            name=f"side_rail_{idx}",
        )

    # Round cross tubes tie the curved side rails together and make the frame read as
    # one welded hand-truck backrest.
    for name, x, z, radius, length in (
        ("toe_crossbar", -0.18, 0.067, 0.022, 0.56),
        ("lower_crossbar", -0.110, 0.42, 0.020, 0.55),
        ("middle_crossbar", -0.055, 0.76, 0.020, 0.55),
        ("upper_crossbar", 0.020, 1.15, 0.020, 0.55),
        ("handle_crossbar", 0.105, 1.43, 0.024, 0.68),
    ):
        frame.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=(x, 0.0, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=red,
            name=name,
        )

    for y, name in ((-0.335, "grip_0"), (0.335, "grip_1")):
        frame.visual(
            Cylinder(radius=0.030, length=0.13),
            origin=Origin(xyz=(0.105, y, 1.43), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark,
            name=name,
        )

    # Horizontal curved cradle bands are shaped to the cylindrical wall of a drum.
    for idx, z in enumerate((0.62, 0.98)):
        cradle_points = [
            (-0.030, -0.245, z),
            (-0.120, -0.155, z),
            (-0.165, 0.0, z),
            (-0.120, 0.155, z),
            (-0.030, 0.245, z),
        ]
        frame.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    cradle_points,
                    radius=0.018,
                    samples_per_segment=16,
                    radial_segments=18,
                    cap_ends=True,
                ),
                f"drum_cradle_band_{idx}",
            ),
            material=red,
            name=f"drum_cradle_{idx}",
        )
        frame.visual(
            Box((0.018, 0.13, 0.030)),
            origin=Origin(xyz=(-0.170, 0.0, z + 0.006)),
            material=dark,
            name=f"rubber_cradle_pad_{idx}",
        )

    # A broad, rounded toe plate sits close to the ground and has a curved front
    # edge/lip so the bottom rim of a cylindrical drum nests into it.
    scoop_profile = [(0.025, -0.315), (0.025, 0.315), (-0.245, 0.315)]
    for i in range(18):
        t = i / 17.0
        y = 0.315 - 0.630 * t
        x = -0.345 + 0.095 * (abs(y) / 0.315) ** 1.8
        scoop_profile.append((x, y))
    scoop_profile.append((-0.245, -0.315))
    frame.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(scoop_profile, 0.036, cap=True, closed=True),
            "curved_scoop_plate",
        ),
        material=plate_gray,
        name="curved_scoop_plate",
    )
    lip_points = [(x, y, 0.052) for x, y in scoop_profile[3:-1]]
    frame.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                lip_points,
                radius=0.013,
                samples_per_segment=6,
                radial_segments=16,
                cap_ends=True,
            ),
            "curved_scoop_lip",
        ),
        material=steel,
        name="curved_scoop_lip",
    )
    for idx, y in enumerate((-0.235, 0.235)):
        frame.visual(
            Box((0.27, 0.045, 0.045)),
            origin=Origin(xyz=(-0.130, y, 0.048)),
            material=red,
            name=f"scoop_side_mount_{idx}",
        )

    # Wheel axle and welded diagonal stays.
    frame.visual(
        Cylinder(radius=0.018, length=0.86),
        origin=Origin(xyz=(0.105, 0.0, 0.200), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="wheel_axle",
    )
    for idx, y in enumerate((-0.235, 0.235)):
        stay_points = [
            (-0.160, y, 0.090),
            (-0.030, y, 0.160),
            (0.112, y, 0.200),
        ]
        frame.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    stay_points,
                    radius=0.016,
                    samples_per_segment=10,
                    radial_segments=16,
                    cap_ends=True,
                ),
                f"axle_stay_{idx}",
            ),
            material=red,
            name=f"axle_stay_{idx}",
        )

    # Rear hinge bosses for the kick-out stabilizers.  The pin is intentionally
    # coaxial with the moving leg barrel, while the red block makes the pivot
    # visibly welded back to the frame.
    for idx, y in enumerate((-0.285, 0.285)):
        frame.visual(
            Box((0.050, 0.080, 0.075)),
            origin=Origin(xyz=(0.130, y, 0.335)),
            material=red,
            name=f"hinge_mount_{idx}",
        )
        for cheek_idx, cheek_y in enumerate((y - 0.044, y + 0.044)):
            frame.visual(
                Box((0.052, 0.018, 0.076)),
                origin=Origin(xyz=(0.181, cheek_y, 0.335)),
                material=red,
                name=f"hinge_cheek_{idx}_{cheek_idx}",
            )
        frame.visual(
            Cylinder(radius=0.012, length=0.105),
            origin=Origin(xyz=(0.185, y, 0.335), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"hinge_pin_{idx}",
        )
        rail_y = -0.235 if y < 0.0 else 0.235
        frame.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    [(-0.125, rail_y, 0.330), (0.020, y, 0.335), (0.130, y, 0.335)],
                    radius=0.014,
                    samples_per_segment=10,
                    radial_segments=16,
                    cap_ends=True,
                ),
                f"hinge_stay_{idx}",
            ),
            material=red,
            name=f"hinge_stay_{idx}",
        )

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.185,
            0.078,
            inner_radius=0.128,
            carcass=TireCarcass(belt_width_ratio=0.68, sidewall_bulge=0.07),
            tread=TireTread(style="block", depth=0.010, count=22, land_ratio=0.56),
            grooves=(TireGroove(center_offset=0.0, width=0.008, depth=0.003),),
            sidewall=TireSidewall(style="rounded", bulge=0.05),
            shoulder=TireShoulder(width=0.008, radius=0.004),
        ),
        "transport_tire",
    )
    rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.126,
            0.082,
            rim=WheelRim(
                inner_radius=0.084,
                flange_height=0.008,
                flange_thickness=0.004,
                bead_seat_depth=0.004,
            ),
            hub=WheelHub(
                radius=0.035,
                width=0.052,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.048, hole_diameter=0.006),
            ),
            face=WheelFace(dish_depth=0.008, front_inset=0.004, rear_inset=0.003),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.004, window_radius=0.016),
            bore=WheelBore(style="round", diameter=0.052),
        ),
        "transport_rim",
    )

    wheel_parts = []
    for idx, y in enumerate((-0.385, 0.385)):
        wheel = model.part(f"wheel_{idx}")
        wheel.visual(
            tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=dark,
            name="tire",
        )
        wheel.visual(
            rim_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=steel,
            name="spoked_rim",
        )
        wheel.visual(
            Cylinder(radius=0.028, length=0.060),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="bearing_sleeve",
        )
        model.articulation(
            f"wheel_spin_{idx}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(0.105, y, 0.200)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=35.0, velocity=10.0),
        )
        wheel_parts.append(wheel)

    for idx, y in enumerate((-0.285, 0.285)):
        leg = model.part(f"stabilizer_{idx}")
        leg.visual(
            Cylinder(radius=0.032, length=0.064),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="hinge_barrel",
        )
        leg.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    [
                        (0.0, 0.0, 0.028),
                        (0.006, 0.0, 0.135),
                        (0.014, 0.0, 0.290),
                        (0.010, 0.0, 0.410),
                    ],
                    radius=0.017,
                    samples_per_segment=10,
                    radial_segments=16,
                    cap_ends=True,
                ),
                f"stabilizer_tube_{idx}",
            ),
            material=red,
            name="folding_tube",
        )
        leg.visual(
            Box((0.070, 0.115, 0.026)),
            origin=Origin(xyz=(0.012, 0.0, 0.410)),
            material=dark,
            name="rubber_foot",
        )
        model.articulation(
            f"stabilizer_hinge_{idx}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=leg,
            origin=Origin(xyz=(0.185, y, 0.335)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=0.0, upper=1.95),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")

    for idx in range(2):
        wheel = object_model.get_part(f"wheel_{idx}")
        stabilizer = object_model.get_part(f"stabilizer_{idx}")
        wheel_joint = object_model.get_articulation(f"wheel_spin_{idx}")
        hinge = object_model.get_articulation(f"stabilizer_hinge_{idx}")

        ctx.check(
            f"wheel_{idx} uses a spinning axle joint",
            wheel_joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(wheel_joint.axis) == (0.0, 1.0, 0.0),
            details=f"type={wheel_joint.articulation_type}, axis={wheel_joint.axis}",
        )
        ctx.expect_within(
            frame,
            wheel,
            axes="xz",
            inner_elem="wheel_axle",
            outer_elem="spoked_rim",
            margin=0.002,
            name=f"wheel_{idx} rim is centered around the axle",
        )
        ctx.allow_overlap(
            frame,
            wheel,
            elem_a="wheel_axle",
            elem_b="bearing_sleeve",
            reason="The rotating wheel has a captured bearing sleeve represented around the fixed axle.",
        )
        ctx.expect_within(
            frame,
            wheel,
            axes="xz",
            inner_elem="wheel_axle",
            outer_elem="bearing_sleeve",
            margin=0.002,
            name=f"wheel_{idx} axle is retained inside bearing sleeve",
        )
        ctx.expect_overlap(
            frame,
            wheel,
            axes="y",
            elem_a="wheel_axle",
            elem_b="bearing_sleeve",
            min_overlap=0.050,
            name=f"wheel_{idx} bearing sleeve stays on axle",
        )

        ctx.allow_overlap(
            frame,
            stabilizer,
            elem_a=f"hinge_pin_{idx}",
            elem_b="hinge_barrel",
            reason="The stabilizer barrel is captured around the fixed hinge pin so the folding leg stays clipped to the frame.",
        )
        ctx.allow_overlap(
            frame,
            stabilizer,
            elem_a=f"hinge_mount_{idx}",
            elem_b="hinge_barrel",
            reason="A small local hinge-block embed represents the welded side clevis retaining the stabilizer barrel.",
        )
        ctx.expect_within(
            frame,
            stabilizer,
            axes="xz",
            inner_elem=f"hinge_pin_{idx}",
            outer_elem="hinge_barrel",
            margin=0.003,
            name=f"stabilizer_{idx} hinge pin remains inside barrel",
        )
        ctx.expect_overlap(
            frame,
            stabilizer,
            axes="y",
            elem_a=f"hinge_pin_{idx}",
            elem_b="hinge_barrel",
            min_overlap=0.045,
            name=f"stabilizer_{idx} hinge barrel overlaps pin length",
        )

        rest_foot = ctx.part_element_world_aabb(stabilizer, elem="rubber_foot")
        with ctx.pose({hinge: 1.95}):
            deployed_foot = ctx.part_element_world_aabb(stabilizer, elem="rubber_foot")
            ctx.expect_within(
                frame,
                stabilizer,
                axes="xz",
                inner_elem=f"hinge_pin_{idx}",
                outer_elem="hinge_barrel",
                margin=0.003,
                name=f"stabilizer_{idx} deployed hinge remains clipped",
            )

        rest_min_z = rest_foot[0][2] if rest_foot is not None else None
        deployed_min_z = deployed_foot[0][2] if deployed_foot is not None else None
        deployed_max_x = deployed_foot[1][0] if deployed_foot is not None else None
        rest_max_x = rest_foot[1][0] if rest_foot is not None else None
        ctx.check(
            f"stabilizer_{idx} folds down and rearward",
            rest_min_z is not None
            and deployed_min_z is not None
            and rest_max_x is not None
            and deployed_max_x is not None
            and deployed_min_z < rest_min_z - 0.22
            and deployed_max_x > rest_max_x + 0.20,
            details=(
                f"rest_min_z={rest_min_z}, deployed_min_z={deployed_min_z}, "
                f"rest_max_x={rest_max_x}, deployed_max_x={deployed_max_x}"
            ),
        )

    return ctx.report()


object_model = build_object_model()
