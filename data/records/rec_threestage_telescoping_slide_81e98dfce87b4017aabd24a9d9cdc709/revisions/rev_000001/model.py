from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_stage_telescoping_slide_study")

    steel = model.material("dark_phosphate_steel", color=(0.09, 0.10, 0.11, 1.0))
    zinc = model.material("brushed_zinc", color=(0.62, 0.65, 0.66, 1.0))
    hard_chrome = model.material("polished_bearing_steel", color=(0.86, 0.88, 0.86, 1.0))
    black = model.material("black_oxide", color=(0.015, 0.016, 0.018, 1.0))
    rubber = model.material("nitrile_wiper", color=(0.01, 0.01, 0.009, 1.0))
    uhmw = model.material("ivory_uhmw_pad", color=(0.86, 0.82, 0.68, 1.0))
    cap_blue = model.material("blued_cover_plate", color=(0.08, 0.13, 0.18, 1.0))
    brass = model.material("brass_label", color=(0.72, 0.55, 0.24, 1.0))

    def box(part, name, size, center, material):
        part.visual(Box(size), origin=Origin(xyz=center), material=material, name=name)

    def cyl_z(part, name, radius, length, center, material):
        part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=center), material=material, name=name)

    def cyl_y(part, name, radius, length, center, material):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=center, rpy=(pi / 2.0, 0.0, 0.0)),
            material=material,
            name=name,
        )

    def add_cover(part, length, width, thickness, x0, z_bottom, screw_radius, screw_length, prefix):
        cover = model.part(f"{prefix}_cover")
        box(
            cover,
            "cover_plate",
            (length, width, thickness),
            (x0 + length / 2.0, 0.0, z_bottom + thickness / 2.0),
            cap_blue,
        )
        box(
            cover,
            "etched_lift_tab",
            (0.040, 0.018, thickness * 1.15),
            (x0 + 0.030, 0.0, z_bottom + thickness * 1.08),
            brass,
        )
        for i, sx in enumerate((x0 + 0.040, x0 + length - 0.040)):
            for j, sy in enumerate((-width * 0.32, width * 0.32)):
                cyl_z(
                    cover,
                    f"screw_{i}_{j}",
                    screw_radius,
                    screw_length,
                    (sx, sy, z_bottom + thickness + screw_length / 2.0),
                    black,
                )
        model.articulation(
            f"{part.name}_to_{prefix}_cover",
            ArticulationType.FIXED,
            parent=part,
            child=cover,
            origin=Origin(),
        )
        return cover

    # Fixed support cradle: a fabricated bed with boxed end brackets and guide pads.
    base = model.part("base_frame")
    box(base, "base_plate", (0.860, 0.240, 0.018), (0.430, 0.0, 0.009), steel)
    for sign, tag in ((1.0, "pos"), (-1.0, "neg")):
        box(base, f"{tag}_side_plate", (0.860, 0.014, 0.142), (0.430, sign * 0.105, 0.088), steel)
        box(base, f"{tag}_foot_flange", (0.860, 0.030, 0.010), (0.430, sign * 0.116, 0.023), steel)
        for k, x in enumerate((0.155, 0.550)):
            box(base, f"{tag}_uhmw_pad_{k}", (0.120, 0.012, 0.050), (x, sign * 0.092, 0.095), uhmw)
        for k, x in enumerate((0.145, 0.345, 0.545, 0.745)):
            cyl_y(base, f"{tag}_side_bolt_{k}", 0.008, 0.005, (x, sign * 0.114, 0.118), black)

    # Open boxed end brackets leave the central slide path clear.
    for x, tag in ((0.018, "rear"), (0.842, "front")):
        box(base, f"{tag}_top_tie", (0.036, 0.240, 0.016), (x, 0.0, 0.166), steel)
        for sign, side in ((1.0, "pos"), (-1.0, "neg")):
            box(base, f"{tag}_{side}_boxed_cheek", (0.036, 0.030, 0.150), (x, sign * 0.105, 0.088), steel)

    # Wiper frame at the mouth of the fixed cradle, clearanced around the outer slide.
    box(base, "front_wiper_top", (0.020, 0.170, 0.008), (0.830, 0.0, 0.157), rubber)
    for sign, tag in ((1.0, "pos"), (-1.0, "neg")):
        box(base, f"front_wiper_{tag}", (0.020, 0.007, 0.092), (0.830, sign * 0.0915, 0.097), rubber)

    # Positive-side stop block is outside the outer tube envelope and captures its rear stop dog.
    box(base, "outer_travel_stop", (0.020, 0.014, 0.034), (0.270, 0.091, 0.095), black)

    # First telescoping member.
    outer = model.part("outer_stage")
    l_outer = 0.680
    box(outer, "outer_top_wall", (l_outer, 0.160, 0.012), (l_outer / 2.0, 0.0, 0.142), zinc)
    box(outer, "outer_bottom_wall", (l_outer, 0.160, 0.012), (l_outer / 2.0, 0.0, 0.048), zinc)
    for sign, tag in ((1.0, "pos"), (-1.0, "neg")):
        box(outer, f"outer_{tag}_web", (l_outer, 0.014, 0.106), (l_outer / 2.0, sign * 0.073, 0.095), zinc)
        box(outer, f"outer_{tag}_race", (l_outer - 0.050, 0.006, 0.013), (l_outer / 2.0, sign * 0.063, 0.095), hard_chrome)
        box(outer, f"outer_{tag}_spacer_pad_0", (0.085, 0.006, 0.042), (0.165, sign * 0.083, 0.095), uhmw)
        box(outer, f"outer_{tag}_spacer_pad_1", (0.085, 0.006, 0.042), (0.485, sign * 0.083, 0.095), uhmw)
        for k, x in enumerate((0.120, 0.245, 0.370, 0.495)):
            outer.visual(
                Sphere(radius=0.0035),
                origin=Origin(xyz=(x, sign * 0.0625, 0.095)),
                material=hard_chrome,
                name=f"outer_{tag}_ball_{k}",
            )
    box(outer, "outer_rear_top_collar", (0.026, 0.168, 0.012), (0.013, 0.0, 0.151), black)
    box(outer, "outer_rear_bottom_collar", (0.026, 0.168, 0.012), (0.013, 0.0, 0.039), black)
    for sign, tag in ((1.0, "pos"), (-1.0, "neg")):
        box(outer, f"outer_rear_{tag}_collar", (0.026, 0.008, 0.106), (0.013, sign * 0.082, 0.095), black)
    box(outer, "outer_front_collar", (0.022, 0.168, 0.118), (l_outer - 0.011, 0.0, 0.095), black)
    box(outer, "outer_front_wiper_top", (0.016, 0.122, 0.006), (l_outer - 0.008, 0.0, 0.130), rubber)
    for sign, tag in ((1.0, "pos"), (-1.0, "neg")):
        box(outer, f"outer_front_wiper_{tag}", (0.016, 0.010, 0.066), (l_outer - 0.008, sign * 0.060, 0.095), rubber)
    box(outer, "outer_stop_dog", (0.020, 0.008, 0.024), (0.030, 0.084, 0.095), black)
    box(outer, "middle_travel_stop", (0.020, 0.006, 0.026), (0.330, 0.063, 0.095), black)
    add_cover(outer, 0.280, 0.080, 0.004, 0.210, 0.148, 0.006, 0.003, "outer")

    # Second telescoping member, nested inside the outer guide channel.
    middle = model.part("middle_stage")
    l_middle = 0.600
    box(middle, "middle_top_wall", (l_middle, 0.105, 0.008), (l_middle / 2.0, 0.0, 0.119), steel)
    box(middle, "middle_bottom_wall", (l_middle, 0.105, 0.008), (l_middle / 2.0, 0.0, 0.071), steel)
    for sign, tag in ((1.0, "pos"), (-1.0, "neg")):
        box(middle, f"middle_{tag}_web", (l_middle, 0.010, 0.056), (l_middle / 2.0, sign * 0.0475, 0.095), steel)
        box(middle, f"middle_{tag}_race", (l_middle - 0.046, 0.0045, 0.010), (l_middle / 2.0, sign * 0.0405, 0.095), hard_chrome)
        box(middle, f"middle_{tag}_spacer_pad_0", (0.070, 0.0075, 0.030), (0.135, sign * 0.05625, 0.095), uhmw)
        box(middle, f"middle_{tag}_spacer_pad_1", (0.070, 0.0075, 0.030), (0.410, sign * 0.05625, 0.095), uhmw)
        for k, x in enumerate((0.115, 0.235, 0.355, 0.475)):
            middle.visual(
                Sphere(radius=0.0030),
                origin=Origin(xyz=(x, sign * 0.0400, 0.095)),
                material=hard_chrome,
                name=f"middle_{tag}_ball_{k}",
            )
    box(middle, "middle_rear_top_collar", (0.022, 0.112, 0.008), (0.011, 0.0, 0.125), black)
    box(middle, "middle_rear_bottom_collar", (0.022, 0.112, 0.008), (0.011, 0.0, 0.065), black)
    for sign, tag in ((1.0, "pos"), (-1.0, "neg")):
        box(middle, f"middle_rear_{tag}_collar", (0.022, 0.006, 0.058), (0.011, sign * 0.0555, 0.095), black)
    box(middle, "middle_front_collar", (0.018, 0.112, 0.066), (l_middle - 0.009, 0.0, 0.095), black)
    box(middle, "middle_front_wiper_top", (0.014, 0.078, 0.005), (l_middle - 0.007, 0.0, 0.113), rubber)
    for sign, tag in ((1.0, "pos"), (-1.0, "neg")):
        box(middle, f"middle_front_wiper_{tag}", (0.014, 0.006, 0.036), (l_middle - 0.007, sign * 0.038, 0.095), rubber)
    box(middle, "middle_stop_dog", (0.020, 0.006, 0.020), (0.030, 0.056, 0.095), black)
    box(middle, "inner_travel_stop", (0.020, 0.0045, 0.020), (0.370, 0.0395, 0.095), black)
    add_cover(middle, 0.200, 0.052, 0.003, 0.160, 0.123, 0.0045, 0.002, "middle")

    # Third telescoping member: a compact rectangular travel bar with exposed pads and a boxed nose.
    inner = model.part("inner_stage")
    l_inner = 0.520
    box(inner, "inner_bar", (l_inner, 0.070, 0.026), (l_inner / 2.0, 0.0, 0.097), zinc)
    box(inner, "inner_top_rib", (l_inner - 0.040, 0.042, 0.006), (l_inner / 2.0, 0.0, 0.113), hard_chrome)
    box(inner, "inner_bottom_rib", (l_inner - 0.040, 0.042, 0.006), (l_inner / 2.0, 0.0, 0.081), hard_chrome)
    for sign, tag in ((1.0, "pos"), (-1.0, "neg")):
        box(inner, f"inner_{tag}_wear_pad_0", (0.070, 0.00325, 0.016), (0.125, sign * 0.036625, 0.097), uhmw)
        box(inner, f"inner_{tag}_wear_pad_1", (0.070, 0.00325, 0.016), (0.360, sign * 0.036625, 0.097), uhmw)
    box(inner, "inner_nose_block", (0.030, 0.074, 0.030), (l_inner - 0.015, 0.0, 0.097), black)
    box(inner, "inner_stop_dog", (0.020, 0.004, 0.018), (0.030, 0.037, 0.095), black)
    add_cover(inner, 0.160, 0.040, 0.002, 0.060, 0.116, 0.0035, 0.001, "inner")

    model.articulation(
        "base_to_outer",
        ArticulationType.PRISMATIC,
        parent=base,
        child=outer,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=520.0, velocity=0.35, lower=0.0, upper=0.220),
        motion_properties=MotionProperties(damping=18.0, friction=9.0),
    )
    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=420.0, velocity=0.35, lower=0.0, upper=0.280),
        motion_properties=MotionProperties(damping=14.0, friction=7.0),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=320.0, velocity=0.35, lower=0.0, upper=0.320),
        motion_properties=MotionProperties(damping=12.0, friction=5.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_frame")
    outer = object_model.get_part("outer_stage")
    middle = object_model.get_part("middle_stage")
    inner = object_model.get_part("inner_stage")
    j0 = object_model.get_articulation("base_to_outer")
    j1 = object_model.get_articulation("outer_to_middle")
    j2 = object_model.get_articulation("middle_to_inner")

    ctx.expect_within(outer, base, axes="yz", margin=0.004, name="outer stage is cradled by base guides")
    ctx.expect_within(middle, outer, axes="yz", margin=0.004, name="middle stage nests inside outer stage")
    ctx.expect_within(inner, middle, axes="yz", margin=0.004, name="inner stage nests inside middle stage")
    ctx.expect_overlap(outer, base, axes="x", min_overlap=0.55, name="outer stage has collapsed retained overlap")
    ctx.expect_overlap(middle, outer, axes="x", min_overlap=0.50, name="middle stage has collapsed retained overlap")
    ctx.expect_overlap(inner, middle, axes="x", min_overlap=0.45, name="inner stage has collapsed retained overlap")

    rest_inner = ctx.part_world_position(inner)
    with ctx.pose({j0: 0.220, j1: 0.280, j2: 0.320}):
        ctx.expect_overlap(outer, base, axes="x", min_overlap=0.40, name="outer stage remains inserted at full travel")
        ctx.expect_overlap(middle, outer, axes="x", min_overlap=0.28, name="middle stage remains inserted at full travel")
        ctx.expect_overlap(inner, middle, axes="x", min_overlap=0.18, name="inner stage remains inserted at full travel")
        extended_inner = ctx.part_world_position(inner)

    ctx.check(
        "nested prismatic joints extend the inner stage forward",
        rest_inner is not None and extended_inner is not None and extended_inner[0] > rest_inner[0] + 0.75,
        details=f"rest_inner={rest_inner}, extended_inner={extended_inner}",
    )

    with ctx.pose({j0: 0.220}):
        ctx.expect_contact(
            outer,
            base,
            elem_a="outer_stop_dog",
            elem_b="outer_travel_stop",
            contact_tol=0.002,
            name="outer stop dog meets fixed travel stop",
        )
    with ctx.pose({j1: 0.280}):
        ctx.expect_contact(
            middle,
            outer,
            elem_a="middle_stop_dog",
            elem_b="middle_travel_stop",
            contact_tol=0.002,
            name="middle stop dog meets outer travel stop",
        )
    with ctx.pose({j2: 0.320}):
        ctx.expect_contact(
            inner,
            middle,
            elem_a="inner_stop_dog",
            elem_b="inner_travel_stop",
            contact_tol=0.002,
            name="inner stop dog meets middle travel stop",
        )

    return ctx.report()


object_model = build_object_model()
