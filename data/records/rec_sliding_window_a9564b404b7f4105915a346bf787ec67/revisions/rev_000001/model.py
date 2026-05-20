from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_sliding_window")

    dark_frame = Material("powder_coated_dark_bronze", rgba=(0.09, 0.075, 0.055, 1.0))
    worn_edges = Material("brushed_stainless_wear_strip", rgba=(0.72, 0.70, 0.64, 1.0))
    glass = Material("slightly_smoked_glass", rgba=(0.55, 0.75, 0.85, 0.36))
    black_rubber = Material("black_epdm_gasket", rgba=(0.015, 0.014, 0.012, 1.0))
    yellow_ptfe = Material("replaceable_yellow_ptfe", rgba=(0.95, 0.72, 0.12, 1.0))
    red_handle = Material("red_service_handle", rgba=(0.78, 0.08, 0.04, 1.0))
    bolt_metal = Material("zinc_plated_fasteners", rgba=(0.62, 0.62, 0.58, 1.0))

    # Fixed service frame: heavy rectangular extrusion, separate front/rear
    # channels, a fixed rear pane, removable wear strips, end stops, and hinge
    # brackets for the service cover.  Coordinates use X = slide direction,
    # Y = service/front depth, Z = height.
    frame = model.part("frame")
    frame.visual(Box((1.60, 0.12, 0.12)), origin=Origin(xyz=(0.0, 0.0, 0.06)), material=dark_frame, name="bottom_rail")
    frame.visual(Box((1.60, 0.12, 0.12)), origin=Origin(xyz=(0.0, 0.0, 0.99)), material=dark_frame, name="top_rail")
    frame.visual(Box((0.08, 0.12, 1.05)), origin=Origin(xyz=(-0.76, 0.0, 0.525)), material=dark_frame, name="jamb_0")
    frame.visual(Box((0.08, 0.12, 1.05)), origin=Origin(xyz=(0.76, 0.0, 0.525)), material=dark_frame, name="jamb_1")

    # Channel guide lips are intentionally proud and replaceable-looking.  The
    # sliding sash runs between front and rear lips with generous side clearance,
    # rather than intersecting the frame.
    frame.visual(Box((1.40, 0.026, 0.085)), origin=Origin(xyz=(0.0, 0.074, 0.162)), material=dark_frame, name="bottom_front_lip")
    frame.visual(Box((1.40, 0.026, 0.085)), origin=Origin(xyz=(0.0, -0.074, 0.162)), material=dark_frame, name="bottom_back_lip")
    frame.visual(Box((1.40, 0.030, 0.095)), origin=Origin(xyz=(0.0, 0.074, 0.884)), material=dark_frame, name="top_front_lip")
    frame.visual(Box((1.40, 0.026, 0.095)), origin=Origin(xyz=(0.0, -0.074, 0.884)), material=dark_frame, name="top_back_lip")
    frame.visual(Box((1.40, 0.020, 0.012)), origin=Origin(xyz=(0.0, 0.033, 0.126)), material=worn_edges, name="stainless_skid_rail")
    frame.visual(Box((1.40, 0.018, 0.010)), origin=Origin(xyz=(0.0, -0.033, 0.929)), material=worn_edges, name="anti_lift_wear_strip")

    # Rear fixed pane and its serviceable rubber glazing.  The mullion bridges
    # into the top and bottom rails and carries the latch keeper.
    frame.visual(Box((0.060, 0.080, 0.830)), origin=Origin(xyz=(-0.035, -0.040, 0.525)), material=dark_frame, name="fixed_mullion")
    frame.visual(Box((0.61, 0.008, 0.70)), origin=Origin(xyz=(-0.390, -0.045, 0.535)), material=glass, name="fixed_glass")
    frame.visual(Box((0.025, 0.020, 0.72)), origin=Origin(xyz=(-0.710, -0.045, 0.535)), material=black_rubber, name="fixed_gasket_0")
    frame.visual(Box((0.025, 0.020, 0.72)), origin=Origin(xyz=(-0.075, -0.045, 0.535)), material=black_rubber, name="fixed_gasket_1")
    frame.visual(Box((0.64, 0.020, 0.025)), origin=Origin(xyz=(-0.390, -0.045, 0.880)), material=black_rubber, name="fixed_gasket_2")
    frame.visual(Box((0.64, 0.020, 0.025)), origin=Origin(xyz=(-0.390, -0.045, 0.190)), material=black_rubber, name="fixed_gasket_3")

    # Service stops and latch keeper are carried by actual brackets tied back
    # to the frame, not by floating pads.
    frame.visual(Box((0.075, 0.090, 0.055)), origin=Origin(xyz=(0.675, 0.105, 0.160)), material=dark_frame, name="closed_stop_bracket")
    frame.visual(Box((0.025, 0.052, 0.070)), origin=Origin(xyz=(0.632, 0.085, 0.160)), material=black_rubber, name="closed_stop_bumper")
    frame.visual(Box((0.080, 0.090, 0.055)), origin=Origin(xyz=(-0.665, 0.052, 0.160)), material=dark_frame, name="open_stop_bracket")
    frame.visual(Box((0.025, 0.052, 0.070)), origin=Origin(xyz=(-0.620, 0.040, 0.160)), material=black_rubber, name="open_stop_bumper")
    frame.visual(Box((0.080, 0.150, 0.040)), origin=Origin(xyz=(-0.005, 0.035, 0.635)), material=dark_frame, name="keeper_bridge")
    frame.visual(Box((0.038, 0.032, 0.120)), origin=Origin(xyz=(0.018, 0.124, 0.635)), material=bolt_metal, name="latch_keeper")

    # Captive screw heads on the replaceable strips and stops.  Each fastener is
    # placed against a rail/strip that connects it into the frame part.
    for i, x in enumerate((-0.54, -0.18, 0.18, 0.54)):
        frame.visual(
            Cylinder(radius=0.008, length=0.008),
            origin=Origin(xyz=(x, 0.046, 0.126), rpy=(pi / 2.0, 0.0, 0.0)),
            material=bolt_metal,
            name=f"skid_screw_{i}",
        )
        frame.visual(
            Cylinder(radius=0.012, length=0.008),
            origin=Origin(xyz=(x, -0.047, 0.936), rpy=(pi / 2.0, 0.0, 0.0)),
            material=bolt_metal,
            name=f"anti_lift_screw_{i}",
        )
    for i, (x, z) in enumerate(((0.670, 0.188), (0.670, 0.132), (-0.665, 0.188), (-0.665, 0.132))):
        frame.visual(
            Cylinder(radius=0.012, length=0.010),
            origin=Origin(xyz=(x, 0.102, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=bolt_metal,
            name=f"stop_screw_{i}",
        )

    # Alternating hinge knuckles for the top maintenance cover.  The frame
    # knuckles sit on welded straps; cover knuckles occupy the gaps.
    for i, x in enumerate((-0.46, 0.46)):
        frame.visual(Box((0.230, 0.040, 0.035)), origin=Origin(xyz=(x, 0.078, 1.000)), material=dark_frame, name=f"cover_hinge_strap_{i}")
        frame.visual(
            Cylinder(radius=0.018, length=0.200),
            origin=Origin(xyz=(x, 0.102, 1.018), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_frame,
            name=f"cover_hinge_knuckle_{i}",
        )
    frame.visual(
        Cylinder(radius=0.008, length=1.58),
        origin=Origin(xyz=(0.0, 0.102, 1.018), rpy=(0.0, pi / 2.0, 0.0)),
        material=bolt_metal,
        name="cover_hinge_pin",
    )

    sash = model.part("sash")
    sash_width = 0.66
    sash_height = 0.78
    sash.visual(Box((sash_width, 0.050, 0.070)), origin=Origin(xyz=(0.0, 0.0, -0.355)), material=dark_frame, name="bottom_rail")
    sash.visual(Box((sash_width, 0.050, 0.070)), origin=Origin(xyz=(0.0, 0.0, 0.355)), material=dark_frame, name="top_rail")
    sash.visual(Box((0.070, 0.050, sash_height)), origin=Origin(xyz=(-0.295, 0.0, 0.0)), material=dark_frame, name="stile_0")
    sash.visual(Box((0.070, 0.050, sash_height)), origin=Origin(xyz=(0.295, 0.0, 0.0)), material=dark_frame, name="stile_1")
    sash.visual(Box((0.500, 0.010, 0.600)), origin=Origin(xyz=(0.0, -0.006, 0.0)), material=glass, name="sliding_glass")
    sash.visual(Box((0.540, 0.018, 0.022)), origin=Origin(xyz=(0.0, 0.001, 0.300)), material=black_rubber, name="sash_gasket_top")
    sash.visual(Box((0.540, 0.018, 0.022)), origin=Origin(xyz=(0.0, 0.001, -0.300)), material=black_rubber, name="sash_gasket_bottom")
    sash.visual(Box((0.022, 0.018, 0.640)), origin=Origin(xyz=(-0.260, 0.001, 0.0)), material=black_rubber, name="sash_gasket_0")
    sash.visual(Box((0.022, 0.018, 0.640)), origin=Origin(xyz=(0.260, 0.001, 0.0)), material=black_rubber, name="sash_gasket_1")

    # Replaceable shoes and anti-lift blocks are partially captured in the sash
    # extrusion, with small clearance to the stationary wear strips.
    for i, (x, shoe_name, block_name, screw_name) in enumerate(
        (
            (-0.220, "wear_shoe_0", "anti_lift_block_0", "shoe_screw_0"),
            (0.220, "wear_shoe_1", "anti_lift_block_1", "shoe_screw_1"),
        )
    ):
        sash.visual(Box((0.120, 0.035, 0.018)), origin=Origin(xyz=(x, 0.033, -0.392)), material=yellow_ptfe, name=shoe_name)
        sash.visual(Box((0.100, 0.028, 0.018)), origin=Origin(xyz=(x, -0.033, 0.378)), material=yellow_ptfe, name=block_name)
        sash.visual(
            Cylinder(radius=0.010, length=0.009),
            origin=Origin(xyz=(x, 0.055, -0.382), rpy=(pi / 2.0, 0.0, 0.0)),
            material=bolt_metal,
            name=screw_name,
        )

    # Chunky pull handle: brackets touch the sash face and the bar, so the
    # handle reads as bolted-on hardware rather than a floating handle.
    sash.visual(Box((0.045, 0.052, 0.046)), origin=Origin(xyz=(0.285, 0.051, 0.115)), material=dark_frame, name="handle_mount_0")
    sash.visual(Box((0.045, 0.052, 0.046)), origin=Origin(xyz=(0.285, 0.051, -0.115)), material=dark_frame, name="handle_mount_1")
    sash.visual(Box((0.040, 0.036, 0.300)), origin=Origin(xyz=(0.285, 0.085, 0.000)), material=red_handle, name="pull_handle")

    # Latch mounting plate and shaft for the articulated service latch.
    sash.visual(Box((0.090, 0.036, 0.095)), origin=Origin(xyz=(-0.292, 0.043, 0.100)), material=dark_frame, name="latch_mount_plate")
    sash.visual(
        Cylinder(radius=0.018, length=0.040),
        origin=Origin(xyz=(-0.292, 0.045, 0.100), rpy=(pi / 2.0, 0.0, 0.0)),
        material=bolt_metal,
        name="latch_pivot_shaft",
    )

    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=0.030, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=bolt_metal,
        name="pivot_boss",
    )
    latch.visual(Box((0.040, 0.030, 0.190)), origin=Origin(xyz=(0.0, 0.018, 0.090)), material=red_handle, name="latch_lever")
    latch.visual(Box((0.070, 0.030, 0.028)), origin=Origin(xyz=(0.020, 0.018, 0.178)), material=red_handle, name="latch_thumb_pad")

    access_cover = model.part("access_cover")
    access_cover.visual(Box((1.36, 0.025, 0.165)), origin=Origin(xyz=(0.0, 0.028, -0.092)), material=dark_frame, name="front_cover_panel")
    for x, leaf_name, knuckle_name in (
        (-0.76, "cover_hinge_leaf_0", "cover_knuckle_0"),
        (0.0, "cover_hinge_leaf_1", "cover_knuckle_1"),
        (0.76, "cover_hinge_leaf_2", "cover_knuckle_2"),
    ):
        access_cover.visual(Box((0.160, 0.018, 0.030)), origin=Origin(xyz=(x, 0.010, -0.030)), material=dark_frame, name=leaf_name)
        access_cover.visual(
            Cylinder(radius=0.018, length=0.160),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_frame,
            name=knuckle_name,
        )
    for i, x in enumerate((-0.52, -0.18, 0.18, 0.52)):
        access_cover.visual(
            Cylinder(radius=0.011, length=0.008),
            origin=Origin(xyz=(x, 0.044, -0.105), rpy=(pi / 2.0, 0.0, 0.0)),
            material=bolt_metal,
            name=f"cover_screw_{i}",
        )

    slide = model.articulation(
        "frame_to_sash",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=sash,
        origin=Origin(xyz=(0.370, 0.0, 0.535)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.45, lower=0.0, upper=0.580),
        motion_properties=MotionProperties(damping=18.0, friction=8.0),
    )
    slide.meta["description"] = "Horizontal sliding sash retained by front/rear channel lips and replaceable wear strips."

    model.articulation(
        "sash_to_latch",
        ArticulationType.REVOLUTE,
        parent=sash,
        child=latch,
        origin=Origin(xyz=(-0.292, 0.085, 0.100)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-0.25, upper=1.10),
        motion_properties=MotionProperties(damping=0.6, friction=0.25),
    )

    model.articulation(
        "frame_to_access_cover",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=access_cover,
        origin=Origin(xyz=(0.0, 0.102, 1.018)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.0, lower=0.0, upper=1.20),
        motion_properties=MotionProperties(damping=1.8, friction=0.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    sash = object_model.get_part("sash")
    latch = object_model.get_part("latch")
    cover = object_model.get_part("access_cover")
    slide = object_model.get_articulation("frame_to_sash")
    latch_joint = object_model.get_articulation("sash_to_latch")
    cover_joint = object_model.get_articulation("frame_to_access_cover")

    for knuckle_name in ("cover_knuckle_0", "cover_knuckle_1", "cover_knuckle_2"):
        ctx.allow_overlap(
            frame,
            cover,
            elem_a="cover_hinge_pin",
            elem_b=knuckle_name,
            reason="The service cover knuckle is intentionally captured around the continuous hinge pin.",
        )
        ctx.expect_within(
            frame,
            cover,
            axes="yz",
            elem_a="cover_hinge_pin",
            elem_b=knuckle_name,
            margin=0.001,
            name=f"{knuckle_name} surrounds the hinge pin",
        )
        ctx.expect_overlap(
            frame,
            cover,
            axes="x",
            elem_a="cover_hinge_pin",
            elem_b=knuckle_name,
            min_overlap=0.09,
            name=f"{knuckle_name} has retained pin engagement",
        )

    ctx.check(
        "sash has service-length travel",
        slide.motion_limits is not None
        and slide.motion_limits.lower == 0.0
        and slide.motion_limits.upper is not None
        and 0.55 <= slide.motion_limits.upper <= 0.62,
        details=f"limits={slide.motion_limits}",
    )

    for pose_name, q in (("closed", 0.0), ("open", 0.58)):
        with ctx.pose({slide: q}):
            ctx.expect_within(
                sash,
                frame,
                axes="x",
                elem_a="wear_shoe_0",
                elem_b="stainless_skid_rail",
                margin=0.002,
                name=f"{pose_name} left shoe remains on skid rail",
            )
            ctx.expect_within(
                sash,
                frame,
                axes="x",
                elem_a="wear_shoe_1",
                elem_b="stainless_skid_rail",
                margin=0.002,
                name=f"{pose_name} right shoe remains on skid rail",
            )
            ctx.expect_overlap(
                sash,
                frame,
                axes="x",
                elem_a="top_rail",
                elem_b="top_front_lip",
                min_overlap=0.50,
                name=f"{pose_name} sash top is retained by top channel",
            )
            ctx.expect_gap(
                frame,
                sash,
                axis="y",
                positive_elem="top_front_lip",
                negative_elem="top_rail",
                min_gap=0.012,
                max_gap=0.060,
                name=f"{pose_name} front channel clearance is robust",
            )
            ctx.expect_gap(
                sash,
                frame,
                axis="y",
                positive_elem="top_rail",
                negative_elem="top_back_lip",
                min_gap=0.012,
                max_gap=0.060,
                name=f"{pose_name} rear channel clearance is robust",
            )

    ctx.expect_gap(
        latch,
        sash,
        axis="y",
        positive_elem="pivot_boss",
        negative_elem="latch_pivot_shaft",
        max_gap=0.002,
        max_penetration=0.0,
        name="latch boss seats on protruding shaft",
    )
    ctx.check(
        "latch has a limited service throw",
        latch_joint.motion_limits is not None
        and latch_joint.motion_limits.lower is not None
        and latch_joint.motion_limits.upper is not None
        and latch_joint.motion_limits.lower < 0.0
        and latch_joint.motion_limits.upper > 0.9,
        details=f"limits={latch_joint.motion_limits}",
    )

    rest_cover_aabb = ctx.part_world_aabb(cover)
    with ctx.pose({cover_joint: 1.0}):
        open_cover_aabb = ctx.part_world_aabb(cover)
    ctx.check(
        "maintenance cover swings outward",
        rest_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[1][1] > rest_cover_aabb[1][1] + 0.035,
        details=f"rest={rest_cover_aabb}, open={open_cover_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
