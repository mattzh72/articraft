from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="usb_flash_drive")

    # Colors
    color_housing = (0.1, 0.1, 0.1, 1.0) # Black plastic
    color_usb_metal = (0.8, 0.8, 0.85, 1.0) # Silver metal
    color_usb_block = (0.1, 0.1, 0.8, 1.0) # Blue USB 3.0 plastic
    color_cover = (0.7, 0.1, 0.1, 1.0) # Red anodized aluminum cover
    color_pin = (0.6, 0.6, 0.6, 1.0) # Metal pin
    
    # ---------------------------------------------------------
    # Drive Body
    # ---------------------------------------------------------
    drive_body = model.part("drive_body")

    # 1. Housing (Black Plastic)
    # The housing is a cylinder at the pivot and a rectangular box extending forward.
    housing_cyl = cq.Workplane("XY").cylinder(height=6, radius=7)
    housing_box = cq.Workplane("XY").center(9, 0).box(18, 14, 6)
    housing = housing_cyl.union(housing_box)
    # Cut hole for the pivot pin
    housing = housing.cut(cq.Workplane("XY").cylinder(height=10, radius=1.6))
    # Add a small chamfer to make it look realistic
    housing = housing.edges("|Z").chamfer(0.5)

    drive_body.visual(
        mesh_from_cadquery(housing, "housing"),
        origin=Origin(),
        color=color_housing,
        name="housing",
    )

    # 2. USB Shell (Silver Metal)
    # Starts at X=18, ends at X=30. Width 12, Height 4.5
    usb_shell = cq.Workplane("YZ").workplane(offset=18).rect(12, 4.5).extrude(12)
    # Hollow out the shell
    usb_cutout = cq.Workplane("YZ").workplane(offset=18).rect(11.4, 3.9).extrude(12)
    usb_shell = usb_shell.cut(usb_cutout)
    # Add the square holes on top and bottom
    holes1 = cq.Workplane("XY").workplane(offset=3).center(24, 3).rect(2.5, 2.5).extrude(-6)
    holes2 = cq.Workplane("XY").workplane(offset=3).center(24, -3).rect(2.5, 2.5).extrude(-6)
    usb_shell = usb_shell.cut(holes1).cut(holes2)

    drive_body.visual(
        mesh_from_cadquery(usb_shell, "usb_shell"),
        origin=Origin(),
        color=color_usb_metal,
        name="usb_shell",
    )

    # 3. USB Contact Block (Blue Plastic)
    # Sits at the bottom of the shell. Shell inner bottom is Z = -1.95. Block height is 1.8.
    usb_block = cq.Workplane("YZ").workplane(offset=18).center(0, -1.05).rect(11.4, 1.8).extrude(11.5)
    
    drive_body.visual(
        mesh_from_cadquery(usb_block, "usb_block"),
        origin=Origin(),
        color=color_usb_block,
        name="usb_block",
    )

    # 4. Pivot Pin (Silver Metal)
    head_top = cq.Workplane("XY").workplane(offset=4.2).cylinder(height=0.4, radius=2.5)
    head_bot = cq.Workplane("XY").workplane(offset=-4.2).cylinder(height=0.4, radius=2.5)
    # Make the shaft slightly larger than the hole in the housing so it intersects
    pin_shaft = cq.Workplane("XY").cylinder(height=8.0, radius=1.65)
    pivot_pin = pin_shaft.union(head_top).union(head_bot)

    drive_body.visual(
        mesh_from_cadquery(pivot_pin, "pivot_pin"),
        origin=Origin(),
        color=color_pin,
        name="pivot_pin",
    )

    # ---------------------------------------------------------
    # Swivel Cover
    # ---------------------------------------------------------
    swivel_cover = model.part("swivel_cover")

    profile = (
        cq.Workplane("XZ")
        .moveTo(32, 4)
        .lineTo(-11, 4)
        .radiusArc((-12, 3), -1)
        .lineTo(-12, -3)
        .radiusArc((-11, -4), -1)
        .lineTo(32, -4)
        .lineTo(32, -3.2)
        .lineTo(-11, -3.2)
        .radiusArc((-11.2, -3), -0.2)
        .lineTo(-11.2, 3)
        .radiusArc((-11, 3.2), -0.2)
        .lineTo(32, 3.2)
        .close()
    )
    cover = profile.extrude(8, both=True)
    # Cut pivot hole
    cover = cover.cut(cq.Workplane("XY").cylinder(height=10, radius=1.6))
    # Cut lanyard hole at the back
    cover = cover.cut(cq.Workplane("XY").center(-9.5, 0).cylinder(height=10, radius=1.5))
    
    # Add a slight waist to the cover to make it look cooler
    # The waist can be a cut from the sides
    waist_cut = cq.Workplane("XY").center(15, 12).cylinder(height=10, radius=4.5)
    waist_cut2 = cq.Workplane("XY").center(15, -12).cylinder(height=10, radius=4.5)
    cover = cover.cut(waist_cut).cut(waist_cut2)

    swivel_cover.visual(
        mesh_from_cadquery(cover, "cover"),
        origin=Origin(),
        color=color_cover,
        name="cover",
    )

    # ---------------------------------------------------------
    # Articulation
    # ---------------------------------------------------------
    model.articulation(
        "cover_pivot",
        ArticulationType.CONTINUOUS,
        parent=drive_body,
        child=swivel_cover,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    body = object_model.get_part("drive_body")
    cover = object_model.get_part("swivel_cover")
    pivot = object_model.get_articulation("cover_pivot")

    # The pivot pin sits inside the cover's pivot hole.
    ctx.allow_overlap(
        body, cover,
        elem_a="pivot_pin",
        elem_b="cover",
        reason="The pivot pin passes through the hole in the cover to act as the hinge."
    )

    # The housing is inside the cover
    ctx.allow_overlap(
        body, cover,
        elem_a="housing",
        elem_b="cover",
        reason="The cover rotates around the housing."
    )

    # The cover intentionally fits over the USB plug when closed
    # and over the housing when rotated.
    ctx.expect_within(
        body, cover,
        axes="z",
        inner_elem="usb_shell",
        outer_elem="cover",
        margin=0.001,
        name="usb shell fits within the Z bounds of the cover"
    )

    # At q=0 (closed), the cover overlaps the USB shell on XY
    ctx.expect_overlap(
        cover, body,
        axes="xy",
        elem_a="cover",
        elem_b="usb_shell",
        min_overlap=0.01,
        name="cover protects the usb plug when closed"
    )

    # At q=pi (open), the cover overlaps the housing on XY, exposing the plug
    with ctx.pose({pivot: 3.14159}):
        ctx.expect_overlap(
            cover, body,
            axes="xy",
            elem_a="cover",
            elem_b="housing",
            min_overlap=0.01,
            name="cover swings back over housing when open"
        )
        # Verify the usb shell is no longer covered by the cover on X axis
        # Cover at q=pi has its front at X=-32 and back at X=12.
        # USB shell is from X=18 to X=30.
        # So they should have a gap on X.
        ctx.expect_gap(
            body, cover,
            axis="x",
            positive_elem="usb_shell",
            negative_elem="cover",
            min_gap=0.005,
            name="usb plug is exposed when cover is open"
        )
    
    return ctx.report()

object_model = build_object_model()
